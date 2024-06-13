#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/settings/settings.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>

#define BUTTON_NODE DT_ALIAS(sw0)

static const struct gpio_dt_spec bouton = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);

static struct gpio_callback button_cb;

#define OP_ONOFF_SET_UNACK BT_MESH_MODEL_OP_2(0x82, 0x03)
#define OP_ONOFF_STATUS    BT_MESH_MODEL_OP_2(0x82, 0x04)

#define NET_KEY { 0xd2, 0xa0, 0xe7, 0x8a, 0x12, 0xd0, 0xf6, 0xc9, \
                  0xa2, 0xb8, 0xe9, 0x38, 0xdb, 0xe4, 0xf5, 0x7c }
#define APP_KEY { 0x3c, 0xde, 0x18, 0xe7, 0xe3, 0xa2, 0xc5, 0x6e, \
                  0x8d, 0x6a, 0x1b, 0x0a, 0x7b, 0x20, 0xd2, 0xa5 }

static const char *const onoff_str[] = { "off", "on" };
static uint8_t tid;

static void prov_complete(uint16_t net_idx, uint16_t addr) {}
static void prov_reset(void) {
    bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

static uint8_t dev_uuid[16];
static const struct bt_mesh_prov prov = {
    .uuid = dev_uuid,
    .output_size = 4,
    .complete = prov_complete,
    .reset = prov_reset,
};

/* Generic OnOff Client */
static int gen_onoff_status(const struct bt_mesh_model *model,
                            struct bt_mesh_msg_ctx *ctx,
                            struct net_buf_simple *buf)
{
    uint8_t present = net_buf_simple_pull_u8(buf);
    printk("OnOff status: %s\n", onoff_str[present]);
    return 0;
}

static const struct bt_mesh_model_op gen_onoff_cli_op[] = {
    {OP_ONOFF_STATUS, BT_MESH_LEN_MIN(1), gen_onoff_status},
    BT_MESH_MODEL_OP_END,
};

static struct bt_mesh_model models[] = {
    BT_MESH_MODEL_CFG_SRV,
    BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op, NULL, NULL),
};

static int gen_onoff_send(bool val)
{
    struct bt_mesh_msg_ctx ctx = {
        .app_idx = models[1].keys[0],
        .addr = BT_MESH_ADDR_ALL_NODES,
        .send_ttl = BT_MESH_TTL_DEFAULT,
    };

    BT_MESH_MODEL_BUF_DEFINE(buf, OP_ONOFF_SET_UNACK, 2);
    bt_mesh_model_msg_init(&buf, OP_ONOFF_SET_UNACK);
    net_buf_simple_add_u8(&buf, val);
    net_buf_simple_add_u8(&buf, tid++);

    printk("Sending OnOff Set: %s\n", onoff_str[val]);
    return bt_mesh_model_send(&models[1], &ctx, &buf, NULL, NULL);
}

static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	static uint8_t net_key[16] = NET_KEY;
    static uint8_t dev_key[16];
    static uint8_t app_key[16] = APP_KEY;
    int err;
	uint16_t addr;

	static bool led_state = false;
	
	if (IS_ENABLED(CONFIG_HWINFO)) {
        addr = sys_get_le16(&dev_uuid[0]) & BIT_MASK(15);
    } else {
        addr = k_uptime_get_32() & BIT_MASK(15);
    }

    if (bt_mesh_is_provisioned()) {
		led_state = !led_state;
        err = gen_onoff_send(led_state);  // Send ON message
		if(err){
			printk("Message send error : %i\n", err);
		}
		return;
    }

	printk("Provisioning with address 0x%04x\n", addr);
    err = bt_mesh_provision(net_key, 0, 0, 0, addr, dev_key);
    if (err) {
        printk("Provisioning failed (err: %d)\n", err);
        return;
    }

    err = bt_mesh_app_key_add(0, 0, app_key);
    if (err) {
        printk("App key add failed (err: %d)\n", err);
        return;
    }
}

static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
    .cid = BT_COMP_ID_LF,
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

int main(void)
{
    int ret;

	gpio_pin_configure_dt(&bouton, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&bouton, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button_cb, button_pressed, BIT(bouton.pin));
    gpio_add_callback(bouton.port, &button_cb);

    printk("Initializing client...\n");

    ret = bt_enable(NULL);
    if (ret) {
        printk("Bluetooth init failed (err %d)\n", ret);
        return -1;
    }

    printk("Bluetooth initialized\n");
    ret = bt_mesh_init(&prov, &comp);
	if (ret) {
        printk("Mesh initialization failed (err %d)\n", ret);
        return -1;
    }

    bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
	printk("Mesh initialized\n");

	while(1){
	}

	return 0;
}
