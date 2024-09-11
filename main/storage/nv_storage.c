#include "storage/nv_storage.h"
#include "typedefs.h"
#include <nvs_flash.h>
#include "esp_log.h"

static const char *TAG = "NVM Storage";

uint8_t storage_save(void *ptr, enum Storage type)
{
    nvs_handle_t nvm_handle;
    esp_err_t ret = nvs_open("storage", NVS_READWRITE, &nvm_handle);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(ret));
        
    } 
    switch (type)
    {
        case CONFIG_DATA:

            config_t *config_ptr = (config_t *)ptr;
            // Struct'i NVM'e yazma
            ret = nvs_set_blob(nvm_handle, "config", config_ptr, sizeof(config_t));
            if (ret != ESP_OK) 
            {
                ESP_LOGE(TAG, "Error (%s) writing config data to NVS!", esp_err_to_name(ret));
                return 0;
            } 
            else 
            {
                ESP_LOGI(TAG, "Configuration written to NVS!");
                nvs_commit(nvm_handle);
            }
            break;

        case ACCEL_CALIB_DATA:
            
            calibration_t *accel_calib_ptr = (calibration_t *)ptr;
            // Struct'i NVM'e yazma
            ret = nvs_set_blob(nvm_handle, "accel_calib", accel_calib_ptr, sizeof(calibration_t));
            if (ret != ESP_OK) 
            {
                ESP_LOGE(TAG, "Error (%s) writing accel calibration data to NVS!", esp_err_to_name(ret));
                return 0;
            } 
            else 
            {
                ESP_LOGI(TAG, "Accel calibration written to NVS!");
                nvs_commit(nvm_handle);
            }
            break;

        case MAG_CALIB_DATA:

            calibration_t *mag_calib_ptr = (calibration_t *)ptr;
            // Struct'i NVM'e yazma
            ret = nvs_set_blob(nvm_handle, "mag_calib", mag_calib_ptr, sizeof(calibration_t));
            if (ret != ESP_OK) 
            {
                ESP_LOGE(TAG, "Error (%s) writing mag calibration data to NVS!", esp_err_to_name(ret));
                return 0;
            } 
            else 
            {
                ESP_LOGI(TAG, "Mag calibration written to NVS!");
                nvs_commit(nvm_handle);
            }
            break;

        case MISSION_DATA:

            waypoint_t *mission_ptr = (waypoint_t *)ptr;
            // Struct'i NVM'e yazma
            ret = nvs_set_blob(nvm_handle, "mission", mission_ptr, sizeof(waypoint_t));
            if (ret != ESP_OK) 
            {
                ESP_LOGE(TAG, "Error (%s) writing mission data to NVS!", esp_err_to_name(ret));
                return 0;
            } 
            else 
            {
                ESP_LOGI(TAG, "Mission written to NVS!\n");
                nvs_commit(nvm_handle);
            }
            break;

        default:
            ESP_LOGE(TAG, "Unknown data type, NVS Storage");
            return 0;
            break;
    }

    nvs_close(nvm_handle);
    return 1;
}


uint8_t storage_read(void *ptr, enum Storage type)
{
    nvs_handle_t nvm_handle;
    size_t required_size;
    esp_err_t ret = nvs_open("storage", NVS_READWRITE, &nvm_handle);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(ret));
        return 0;
    }

    switch (type)
    {
        case CONFIG_DATA:
            
            ret = nvs_get_blob(nvm_handle, "config", NULL, &required_size);
            if (ret == ESP_OK && required_size == sizeof(config_t)) 
            {
                ret = nvs_get_blob(nvm_handle, "config", ptr, &required_size);
                if (ret != ESP_OK) 
                {
                    ESP_LOGI(TAG, "Custom config not found! (%s)", esp_err_to_name(ret));
                    nvs_close(nvm_handle);
                    return 0;
                }
            } 
            else 
            {
                ESP_LOGI(TAG, "Custom config not found! (%s)", esp_err_to_name(ret));
                nvs_close(nvm_handle);
                return 0;
            }
            break;

        case ACCEL_CALIB_DATA:

            ret = nvs_get_blob(nvm_handle, "accel_calib", NULL, &required_size);
            if (ret == ESP_OK && required_size == sizeof(calibration_t)) 
            {
                ret = nvs_get_blob(nvm_handle, "accel_calib", ptr, &required_size);
                if (ret != ESP_OK) 
                {
                    ESP_LOGI(TAG, "Accel calibration data not found! (%s)", esp_err_to_name(ret));
                    nvs_close(nvm_handle);
                    return 0;
                } 
            } 
            else 
            {
                ESP_LOGI(TAG, "Accel calibration data not found! (%s)", esp_err_to_name(ret));
                nvs_close(nvm_handle);
                return 0;
            }
            break;

        case MAG_CALIB_DATA:

            ret = nvs_get_blob(nvm_handle, "mag_calib", NULL, &required_size);
            if (ret == ESP_OK && required_size == sizeof(calibration_t)) 
            {
                ret = nvs_get_blob(nvm_handle, "mag_calib", ptr, &required_size);
                if (ret != ESP_OK) 
                {
                    ESP_LOGI(TAG, "Mag calibration data not found! (%s)", esp_err_to_name(ret));
                    nvs_close(nvm_handle);
                    return 0;
                } 
            } 
            else 
            {
                ESP_LOGI(TAG, "Mag calibration data not found! (%s)", esp_err_to_name(ret));
                nvs_close(nvm_handle);
                return 0;
            }
            break;

        case MISSION_DATA:

            ret = nvs_get_blob(nvm_handle, "mission", NULL, &required_size);
            if (ret == ESP_OK && required_size == sizeof(waypoint_t)) 
            {
                ret = nvs_get_blob(nvm_handle, "mission", ptr, &required_size);
                if (ret != ESP_OK) 
                {
                    ESP_LOGI(TAG, "Mission data not found! (%s)", esp_err_to_name(ret));
                    nvs_close(nvm_handle);
                    return 0;
                } 
            } 
            else 
            {
                ESP_LOGI(TAG, "Mission data not found! (%s)", esp_err_to_name(ret));
                nvs_close(nvm_handle);
                return 0;
            }
            break;

        default:
            ESP_LOGE(TAG, "Unknown data type, NVS Storage");
            nvs_close(nvm_handle);
            return 0;
            break;
    }
    nvs_close(nvm_handle);
    return 1;
}
