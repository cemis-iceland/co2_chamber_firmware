#ifndef WEBSERVER_H20220712
#define WEBSERVER_H20220712
#include "config.h"
#include <freertos/semphr.h>

/* Serves a web form that allows a connected user to configure the chamber
 * and download data.
 * Once the form is submitted, the task shuts down the web server and gives the 
 * web_setup_done semaphore to indicate that it should be deleted.
 */
void web_setup_task(void* param);

/* Type of the void* parameter that should be passed to web_setup_task via
 * "pvParameters" in xTaskCreate.  */
struct web_setup_task_params_t {
  Config* config; // Config object to update with web form data
  SemaphoreHandle_t web_setup_done; // Semaphore given once task is ready to die
  uint32_t timeout; // Time in seconds after which server will shut down
};

#endif // WEBSERVER_H20220712