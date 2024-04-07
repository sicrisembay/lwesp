/*
 * Home assistant sensor and switch control demo
 */
#include "lwesp/apps/lwesp_mqtt_client_api.h"
#include "lwesp/lwesp_mem.h"
#include "math.h"
#include "mqtt_client_api.h"

/**
 * \brief           Connection information for MQTT CONNECT packet
 */
static const lwesp_mqtt_client_info_t mqtt_client_info = {
    .keep_alive = 10,

    /* Server login data */
    //.user = "8a215f70-a644-11e8-ac49-e932ed599553",
    //.pass = "26aa943f702e5e780f015cd048a91e8fb54cca28",

    /* Device identifier address */
    .id = "123869f5a20-af9c-11e9-b01f-db5cf74e7fb7",
};

static char mqtt_topic_str[1024];  /*!< Topic string */
static char mqtt_topic_data[1024]; /*!< Data string */

/**
 * \brief           MQTT client API thread
 * \param[in]       arg: User argument
 */
void
lwesp_mqtt_client_api_ha_thread(void const* arg) {
    lwesp_mqtt_client_api_p client;
    lwesp_mqtt_conn_status_t conn_status;
    lwesp_mqtt_client_api_buf_p buf;
    lwespr_t res;

    LWESP_UNUSED(arg);

    /* Create new MQTT API */
    if ((client = lwesp_mqtt_client_api_new(1024, 1024)) == NULL) {
        goto terminate;
    }

    while (1) {
        /* Make a connection */
        printf("Joining MQTT server\r\n");

        /* Try to join */
        conn_status = lwesp_mqtt_client_api_connect(client, "192.168.1.16", 1883, &mqtt_client_info);
        if (conn_status == LWESP_MQTT_CONN_STATUS_ACCEPTED) {
            printf("Connected and accepted!\r\n");
            printf("Client is ready to subscribe and publish to new messages\r\n");
        } else {
            printf("Connect API response: %d\r\n", (int)conn_status);
            lwesp_delay(5000);
            continue;
        }

#define SENSOR_DESC_1                                                                                                  \
    "{"                                                                                                                \
    "    \"~\": \"homeassistant/mycustomsensorT%02u\","                                                                \
    "    \"name\": \"Temp sens %02u\","                                                                                \
    "    \"object_id\": \"temp_%02u\","                                                                                \
    "    \"unique_id\": \"tempsensor_unit_%02u\","                                                                     \
    "    \"device_class\": \"temperature\","                                                                           \
    "    \"state_topic\": \"~/state\","                                                                                \
    "    \"unit_of_measurement\": \"°C\","                                                                            \
    "    \"value_template\": \"{{ value_json.temperature }}\","                                                        \
    "    \"device\": {"                                                                                                \
    "        \"identifiers\": ["                                                                                       \
    "            \"switch_unit_DEADBEEF\""                                                                             \
    "        ],"                                                                                                       \
    "        \"name\": \"8R-1L-1TS-1235CD4\","                                                                         \
    "        \"model\": \"8R-1L-1TS\","                                                                                \
    "        \"manufacturer\": \"MaJerle\","                                                                           \
    "        \"serial_number\": \"1235CD4-ADFEFF-DFDD-E543654-1234344343\","                                           \
    "        \"sw_version\": \"1.2.3\","                                                                               \
    "        \"hw_version\": \"4.5.6\""                                                                                \
    "    }"                                                                                                            \
    "}"

#define RELAY_DESC_1                                                                                                   \
    "{"                                                                                                                \
    "    \"~\": \"homeassistant/mycustomswitch%02u\","                                                                 \
    "    \"name\": \"Relay %02u\","                                                                                    \
    "    \"object_id\": \"relay_%02u\","                                                                               \
    "    \"unique_id\": \"switch_unit_%02u\","                                                                         \
    "    \"device_class\": \"switch\","                                                                                \
    "    \"state_topic\": \"~/state\","                                                                                \
    "    \"command_topic\": \"~/set\","                                                                                \
    "    \"device\": {"                                                                                                \
    "        \"identifiers\": ["                                                                                       \
    "            \"switch_unit_DEADBEEF\""                                                                             \
    "        ]"                                                                                                        \
    "    }"                                                                                                            \
    "}"

#define TRIAC_DESC_1                                                                                                   \
    "{"                                                                                                                \
    "    \"~\": \"homeassistant/mycustomtriac%02u\","                                                                  \
    "    \"name\": \"Triac %02u\","                                                                                    \
    "    \"object_id\": \"triac_%02u\","                                                                               \
    "    \"unique_id\": \"triac_unit_%02u\","                                                                          \
    "    \"device_class\": \"light\","                                                                                 \
    "    \"state_topic\": \"~/state\","                                                                                \
    "    \"command_topic\": \"~/set\","                                                                                \
    "    \"brightness\": true,"                                                                                        \
    "    \"brightness_command_topic\": \"~/brightness\","                                                              \
    "    \"brightness_state_topic\": \"~/brightness_state\","                                                          \
    "    \"icon\": \"mdi:lightbulb-on-50\","                                                                           \
    "    \"device\": {"                                                                                                \
    "        \"identifiers\": ["                                                                                       \
    "            \"switch_unit_DEADBEEF\""                                                                             \
    "        ]"                                                                                                        \
    "    }"                                                                                                            \
    "}"

#define LIGHT_DESC_1                                                                                                   \
    "{"                                                                                                                \
    "    \"~\": \"homeassistant/mycustomLightT%02u\","                                                                 \
    "    \"name\": \"RGB strip %02u\","                                                                                \
    "    \"object_id\": \"rgbstrip_%02u\","                                                                            \
    "    \"unique_id\": \"light_unit_%02u\","                                                                          \
    "    \"device_class\": \"light\","                                                                                 \
    "    \"state_topic\": \"~/state\","                                                                                \
    "    \"command_topic\": \"~/set\","                                                                                \
    "    \"brightness\": true,"                                                                                        \
    "    \"brightness_command_topic\": \"~/brightness\","                                                              \
    "    \"brightness_state_topic\": \"~/brightness_state\","                                                          \
    "    \"color_temp_command_topic\": \"~/color_temp\","                                                              \
    "    \"color_temp_state_topic\": \"~/color_temp_state\","                                                          \
    "    \"color_mode_command_topic\": \"~/color_mode\","                                                              \
    "    \"color_mode_state_topic\": \"~/color_mode_state\","                                                          \
    "    \"rgb_command_topic\": \"~/rgb\","                                                                            \
    "    \"rgb_state_topic\": \"~/rgb_state\","                                                                        \
    "    \"rgb\": true,"                                                                                               \
    "    \"device\": {"                                                                                                \
    "        \"identifiers\": ["                                                                                       \
    "            \"switch_unit_DEADBEEF\""                                                                             \
    "        ]"                                                                                                        \
    "    }"                                                                                                            \
    "}"

#if 1
        /* Send temperature sensor including device data */
        for (size_t i = 0; i < 4; ++i) {
            uint32_t id = (i + 1);

            sprintf(mqtt_topic_str, "homeassistant/sensor/mycustomsensorT%02u/config", (unsigned)id);
            sprintf(mqtt_topic_data, SENSOR_DESC_1, (unsigned)id, (unsigned)id, (unsigned)id, (unsigned)id);
            lwesp_mqtt_client_api_publish(client, mqtt_topic_str, mqtt_topic_data, strlen(mqtt_topic_data),
                                          LWESP_MQTT_QOS_AT_LEAST_ONCE, 1);
        }
#endif
#if 1
        /* Send light */
        for (size_t i = 0; i < 4; ++i) {
            uint32_t id = (i + 1);

            sprintf(mqtt_topic_str, "homeassistant/light/mycustomLightT%02u/config", (unsigned)id);
            sprintf(mqtt_topic_data, LIGHT_DESC_1, (unsigned)id, (unsigned)id, (unsigned)id, (unsigned)id);
            lwesp_mqtt_client_api_publish(client, mqtt_topic_str, mqtt_topic_data, strlen(mqtt_topic_data),
                                          LWESP_MQTT_QOS_AT_LEAST_ONCE, 1);
        }
#endif
#if 1
        /* Send switch entities */
        for (size_t i = 0; i < 4; ++i) {
            uint32_t id = (i + 1);

            sprintf(mqtt_topic_str, "homeassistant/switch/switch_unique_%02u/config", (unsigned)id);
            sprintf(mqtt_topic_data, RELAY_DESC_1, (unsigned)id, (unsigned)id, (unsigned)id, (unsigned)id);
            lwesp_mqtt_client_api_publish(client, mqtt_topic_str, mqtt_topic_data, strlen(mqtt_topic_data),
                                          LWESP_MQTT_QOS_AT_LEAST_ONCE, 1);
        }
#endif
#if 1
        /* Send triac entities */
        for (size_t i = 0; i < 4; ++i) {
            uint32_t id = (i + 1);

            sprintf(mqtt_topic_str, "homeassistant/light/triac_unique_%02u/config", (unsigned)id);
            sprintf(mqtt_topic_data, TRIAC_DESC_1, (unsigned)id, (unsigned)id, (unsigned)id, (unsigned)id);
            lwesp_mqtt_client_api_publish(client, mqtt_topic_str, mqtt_topic_data, strlen(mqtt_topic_data),
                                          LWESP_MQTT_QOS_AT_LEAST_ONCE, 1);
        }
#endif

        /* Subscribe to topics */
        if (lwesp_mqtt_client_api_subscribe(client, "homeassistant/#", LWESP_MQTT_QOS_AT_LEAST_ONCE) == lwespOK) {
            printf("Subscribed to topic\r\n");
        } else {
            printf("Problem subscribing to topic!\r\n");
        }

        while (1) {
            static uint32_t sin_counter = 0;

            /* Receive MQTT packet with 1000ms timeout */
            if ((res = lwesp_mqtt_client_api_receive(client, &buf, 3000)) == lwespOK) {
                if (buf != NULL) {
                    printf("Publish received!\r\n");
                    printf("Topic: %s, payload: %s\r\n", buf->topic, buf->payload);
                    lwesp_mqtt_client_api_buf_free(buf);
                    buf = NULL;
                }
            } else if (res == lwespCLOSED) {
                printf("MQTT connection closed!\r\n");
                break;
            } else if (res == lwespTIMEOUT) {
                printf("Timeout on MQTT receive function. Manually publishing.\r\n");

                /* Publish data on channel 1 */
                for (size_t i = 0; i < 4; ++i) {
                    uint32_t id = i + 1;

                    sprintf(mqtt_topic_str, "homeassistant/mycustomsensorT%02u/state", (unsigned)id);
                    sprintf(mqtt_topic_data, "{\"temperature\":%.3f}",
                            21.0f + (float)2.0f * sinf(2.0f * 3.14f * (float)++sin_counter * 100.0f));
                    lwesp_mqtt_client_api_publish(client, mqtt_topic_str, mqtt_topic_data, strlen(mqtt_topic_data),
                                                  LWESP_MQTT_QOS_AT_LEAST_ONCE, 1);
                }
            }
        }
        //goto terminate;
    }

terminate:
    lwesp_mqtt_client_api_delete(client);
    printf("MQTT client thread terminate\r\n");
    lwesp_sys_thread_terminate(NULL);
}
