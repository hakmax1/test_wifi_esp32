#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"


#include "esp_log.h"

//server
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

//console
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"

#include "uri_parser.h"

#define BUF_SIZE (1024)

#define BLINK_GPIO CONFIG_BLINK_GPIO

#define EXAMPLE_ESP_WIFI_MODE_AP   "STA" //TRUE:AP FALSE:STA
#define EXAMPLE_ESP_WIFI_SSID      "atletka_service"
#define EXAMPLE_ESP_WIFI_PASS      "f4g_GROUP"
//#define EXAMPLE_ESP_WIFI_SSID      "MaksHome"
//#define EXAMPLE_ESP_WIFI_PASS      "76543210"
#define EXAMPLE_MAX_STA_CONN       10

const int WIFI_CONNECTED_BIT = BIT0;


/* Constants that aren't configurable in menuconfig */
//#define WEB_SERVER "httpbin.org"
#define WEB_SERVER "52.23.126.223"

#define WEB_PORT 80
#define WEB_URL "http://httpbin.org/get"


static const char *REQUEST = "GET " WEB_URL " HTTP/1.0\r\n"
    "Host: "WEB_SERVER"\r\n"
    "User-Agent: esp-idf/1.0 esp32\r\n"
    "\r\n";

static const char *TAG = "simple wifi";

static EventGroupHandle_t wifi_event_group;



esp_err_t event_handler(void *ctx, system_event_t *event)
{
	switch(event->event_id) {
	    case SYSTEM_EVENT_STA_START:
	    	ESP_LOGI("event_handler", "call esp_wifi_connect()");
	        esp_wifi_connect();
	        break;
	    case SYSTEM_EVENT_STA_GOT_IP:
	        ESP_LOGI("event_handler", "got ip:%s",
	                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
	        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
	        break;
	    case SYSTEM_EVENT_AP_STACONNECTED:
	        ESP_LOGI("event_handler", "station:"MACSTR" join, AID=%d",
	                 MAC2STR(event->event_info.sta_connected.mac),
	                 event->event_info.sta_connected.aid);
	        break;
	    case SYSTEM_EVENT_AP_STADISCONNECTED:
	        ESP_LOGI("event_handler", "station:"MACSTR"leave, AID=%d",
	                 MAC2STR(event->event_info.sta_disconnected.mac),
	                 event->event_info.sta_disconnected.aid);
	        break;
	    case SYSTEM_EVENT_STA_DISCONNECTED:
	    	/* This is a workaround as ESP32 WiFi libs don't currently
	    	           auto-reassociate. */
	        esp_wifi_connect();
	        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
	        break;
	    default:
	        break;
	    }
	    return ESP_OK;
}

void initialize_nvs()
{
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
	    ESP_ERROR_CHECK( nvs_flash_erase() );
	    err = nvs_flash_init();
	}
	ESP_ERROR_CHECK(err);
}
void initialise_wifi()
{


    // create an LwIP core task and initialize LwIP-related work
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    //create a system Event task and initialize an application event’s
    //callback function. In the scenario above,
    //the application event’s callback function
    //does nothing but relaying the event to the application task.
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    //create the Wi-Fi driver task and initialize the Wi-Fi driver
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

void initialize_console()
{
	/* Disable buffering on stdin and stdout */
	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stdout, NULL, _IONBF, 0);

	    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
	esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
	/* Move the caret to the beginning of the next line on '\n' */
	esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

	/* Install UART driver for interrupt-driven reads and writes */
	ESP_ERROR_CHECK( uart_driver_install(CONFIG_CONSOLE_UART_NUM,
	        256, 0, 0, NULL, 0) );

	    /* Tell VFS to use UART driver */
	esp_vfs_dev_uart_use_driver(CONFIG_CONSOLE_UART_NUM);

	    /* Initialize the console */
	esp_console_config_t console_config = {
	        .max_cmdline_args = 8,
	        .max_cmdline_length = 256,
#if CONFIG_LOG_COLORS
	        .hint_color = atoi(LOG_COLOR_CYAN)
#endif
	};
	ESP_ERROR_CHECK( esp_console_init(&console_config) );


	/* Configure linenoise line completion library */
	/* Enable multiline editing. If not set, long commands will scroll within
	 * single line.
	 */
	linenoiseSetMultiLine(1);

	/* Tell linenoise where to get command completions and hints */
	linenoiseSetCompletionCallback(&esp_console_get_completion);
	linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);

	/* Set command history size */
	linenoiseHistorySetMaxLen(100);

#if CONFIG_STORE_HISTORY
	/* Load command history from filesystem */
	linenoiseHistoryLoad(HISTORY_PATH);
#endif
}

/** Arguments used by 'join' function */
static struct {
    struct arg_str *req;
    struct arg_str *url;
    struct arg_str *body;
    struct arg_end *end;
} http_args;

static int test_http_req(int argc, char** argv)
{
	uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
	parsed_uri_t* pars;
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[64];

	int nerrors = arg_parse(argc, argv, (void**) &http_args);
	if (nerrors != 0) {
	    arg_print_errors(stderr, http_args.end, argv[0]);
	    //return 1;
	}
	else
	{
		/*
		printf("test_http_req nerrors == 0\n");
		printf("req =  %s \n", http_args.req->sval[0]);
		printf("url =  %s \n", http_args.url->sval[0]);
		printf("body =  %s \n", http_args.body->sval[0]);
		*/
		pars = parse_uri(http_args.url->sval[0]);

		strcpy((char*)data,"");
		if( strcmp(http_args.req->sval[0],"GET") == 0)
		{
			strcat((char*)data,"GET ");
		}
		else if(strcmp(http_args.req->sval[0],"POST") == 0)
		{
			strcat((char*)data,"POST ");
			//проверить тело post
		}
		else
		{
			printf("Incorrect REQ =  %s \n", http_args.req->sval[0]);
			return 1;
		}

		//printf((char*)data);
		strcat((char*)data,http_args.url->sval[0]);
		//printf((char*)data);
		strcat((char*)data," HTTP/1.0\r\n");
		//printf((char*)data);	"52.23.126.223"
		//strcat((char*)data,"Host: httpbin.org\r\n");
		strcat((char*)data,"Host: ");
		strcat((char*)data,pars->host);
		strcat((char*)data,"\r\n");
		//printf((char*)data);
		//strcpy((char*)data,http_args.url->sval[0]);
		strcat((char*)data,"User-Agent: esp-idf/1.0 esp32\r\n");
		//printf((char*)data);
		strcat((char*)data,"\r\n");
		/*
		printf("-------REQ-----------\r\n");
		printf((char*)data);
		//parse_uri_info(pars);
		printf("-------REQ-----------\r\n");
		*/
		free_parsed_uri(pars);

		int err = getaddrinfo(WEB_SERVER, "80", &hints, &res);

		if(err != 0 || res == NULL) {
		    ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
		    vTaskDelay(1000 / portTICK_PERIOD_MS);
		    return 1;
		    //continue;
		}
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
//        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            return 1;
            //continue;
        }
//        ESP_LOGI(TAG, "... allocated socket");
        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            return 1;
            //continue;
        }
//        ESP_LOGI(TAG, "... connected");
        freeaddrinfo(res);
        //if (write(s, REQUEST, strlen(REQUEST)) < 0) {
        if (write(s, data, strlen(REQUEST)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            return 1;
            //continue;
        }
//        ESP_LOGI(TAG, "... socket send success");

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            return 1;
            //continue;
        }
//        ESP_LOGI(TAG, "... set socket receiving timeout success");

        /* Read HTTP response */
        do {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }
        } while(r > 0);
        printf("\r\n");

//        ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d\r\n", r, errno);
        close(s);
        free(data);
	}
	return 0;
}
/*
 * 	Регистрируем команду
 * */
void register_http_req()
{
	http_args.req = arg_str1(NULL, NULL, "<REQ>", "GET or POST request");
	http_args.url = arg_str1(NULL, NULL, "<URL>", "URL - remote server address");
	http_args.body = arg_str0(NULL, NULL, "<BODY>", "BODY - request body in case of POST");
	http_args.end = arg_end(2);

    const esp_console_cmd_t http_req_cmd = {
        .command = "http",
        .help = "Join WiFi AP as a station",
        .hint = NULL,
        .func = &test_http_req,
        .argtable = &http_args
    };

    ESP_ERROR_CHECK( esp_console_cmd_register(&http_req_cmd) );
}

/**
 *
 * 		!!!	Необходимо дождаться подключения по WiFi
 *
 * */

void app_main(void)
{
	printf("This is my first program on C from ESP32!\n");
	//Инициализируем nvs
	initialize_nvs();
	//Инициализируем wifi
	initialise_wifi();
	// init console
	initialize_console();
	/* Register commands */
	register_http_req();

	esp_console_register_help_command();

	int probe_status = linenoiseProbe();
	if (probe_status) { /* zero indicates success */
	    printf("\n"
	          "Your terminal application does not support escape sequences.\n"
	          "Line editing and history features are disabled.\n"
	          "On Windows, try using Putty instead.\n");
	    linenoiseSetDumbMode(1);
	}
	const char* prompt = "test> ";

	linenoiseClearScreen();
	printf("Wait connection to WiFi\n");
	xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
	                            false, true, portMAX_DELAY);
	linenoiseClearScreen();


	//Ждем подключения по WiFi

	//parsed_uri_t* pars;
	//pars = parse_uri("http://httpbin.org/get");
	//printf("scheme %s",pars->scheme);
	//printf("host \n");
	//printf(pars->host);
	//printf("-----\n");
	//printf("query %s",pars->query);
	//printf("_uri %s",pars->_uri);
    /* Main loop */
    while(true) {
        /* Get a line using linenoise.
         * The line is returned when ENTER is pressed.
         */
        char* line = linenoise(prompt);
        if (line == NULL) { /* Ignore empty lines */
            continue;
        }
        linenoiseClearScreen();
        printf(prompt);
        printf(line);
        printf("\n");
        /* Add the command to the history */
        linenoiseHistoryAdd(line);
#if CONFIG_STORE_HISTORY
        /* Save command history to filesystem */
        linenoiseHistorySave(HISTORY_PATH);
#endif

        /* Try to run the command */
        int ret;
        esp_err_t err = esp_console_run(line, &ret);
        if (err == ESP_ERR_NOT_FOUND) {
            printf("Unrecognized command\n");

        } else if (err == ESP_ERR_INVALID_ARG) {
            // command was empty
        } else if (err == ESP_OK && ret != ESP_OK) {
            printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(err));
        } else if (err != ESP_OK) {
            printf("Internal error: %s\n", esp_err_to_name(err));
        }
        /* linenoise allocates line buffer on the heap, so need to free it */
        linenoiseFree(line);
    }

/*

    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    int level = 0;
    while (true) {
        gpio_set_level(GPIO_NUM_4, level);
        level = !level;
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
    */
}

