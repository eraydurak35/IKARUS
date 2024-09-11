#include "storage/blackbox.h"
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "typedefs.h"

static const char *TAG = "Blackbox";
static FILE* file_handler = NULL;
static flight_t *flight_ptr = NULL;
static imu_t *imu_ptr = NULL;
static void blackbox_create_open_to_write_bin_file();
static uint8_t blackbox_open_to_read_bin_file();
static void blackbox_close_bin_file();
static void write_data_to_bin_file();

typedef struct
{
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} blackbox_t;

static blackbox_t blackbox;
static const char *header = "\ngyro X dps,gyro Y dps,gyro Z dps\n";
static size_t free_bytes_for_blackbox_file = 0;
static size_t written_bytes_to_blackbox_file = 0;

// SPIFFS dosya sistemini başlat
void blackbox_init(flight_t *flt, imu_t *imu)
{
    flight_ptr = flt;
    imu_ptr = imu;

    ESP_LOGI(TAG, "Initializing SPIFFS for blackbox");

    esp_vfs_spiffs_conf_t conf = 
    {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 1,
      .format_if_mount_failed = true
    };
    
    // SPIFFS dosya sistemini başlat
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) 
    {
        if (ret == ESP_FAIL) ESP_LOGE(TAG, "Failed to mount or format filesystem");
        else if (ret == ESP_ERR_NOT_FOUND) ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        else ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        return;
    }

    // SPIFFS dosya sistemi boyutu hakkında bilgi al
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        return;
    }

    // Blackbox için kullanılabilir dosya boyutunu hesapla
    free_bytes_for_blackbox_file = total - used;
    long blackbox_file_size = 0;
    // Eğer zaten kayıtlı bir blackbox.bin dosyası varsa boyutunu öğrenmek için aç
    if (blackbox_open_to_read_bin_file())
    {
        // Dosya işaretçisini sona taşı
        fseek(file_handler, 0, SEEK_END);
        // Dosya işaretçisinin bulunduğu konumu al (dosya boyutu)
        blackbox_file_size = ftell(file_handler);
        // Yeni veri blackbox.bin dosyasının üzerine yazılacağı için bu boyutu da kullanılabilir alan olarak dahil et
        free_bytes_for_blackbox_file += blackbox_file_size;
        // Dosyayı kapat
        blackbox_close_bin_file();
    }

    ESP_LOGI(TAG, "Partition size: total bytes: %d", total);
    ESP_LOGI(TAG, "Usable bytes for blackbox: %d", free_bytes_for_blackbox_file);
    ESP_LOGI(TAG, "Current blackbox file size: %ld", blackbox_file_size);
}

// Bu fonksiyon IMU yapısındaki gyro açısal hızlarını kaydeder
// Kayıt FLIGHT yapısı içerisindeki "arm_status" değişkeninin 1 olduğu anda başlar
// Bu değişken 0 olduğu anda kayıt biter
void blackbox_save()
{
    static uint8_t prev_arm_state = 0;

    // Arm olduğu anda dosya oluştur ve yazmak için aç
    if (flight_ptr->arm_status == 1 && prev_arm_state == 0)
    {
        blackbox_create_open_to_write_bin_file();
        written_bytes_to_blackbox_file = 0;
    }
    // Disarm olduğu anda dosyayı kapat
    else if (flight_ptr->arm_status == 0 && prev_arm_state == 1)
    {
        blackbox_close_bin_file();
    }
    // flight_ptr->arm_status = 1 olduğu sürece ve yeterli alan varsa kayıt et
    if (flight_ptr->arm_status == 1 && written_bytes_to_blackbox_file < free_bytes_for_blackbox_file)
    {
        write_data_to_bin_file();
        written_bytes_to_blackbox_file += sizeof(blackbox_t);
    }

    prev_arm_state = flight_ptr->arm_status;
}

// "blackbox.bin" dosyasından veri okumak için aç
// Dosya varsa 1 yoksa veya açılamadıysa 0 döndürür.
uint8_t blackbox_open()
{
    return blackbox_open_to_read_bin_file();
}
// "blackbox.bin" dosyasından "blackbox_t" yapısı boyutunda veri oku ve blackbox yapısına ata
// Okunan değerileri virgül ile ayrılmış olarak satır satır konsola yazdır.
// Okunacak veri varken 1, dosyanın sonuna gelindiyse 0 döndürür
uint8_t blackbox_print()
{   
    // okunacak veri varken 1, dosyanın sonuna gelindiyse 0 döndürür
    uint8_t ret = fread(&blackbox, sizeof(blackbox_t), 1, file_handler);
    if (ret == 1)
    {
        printf("%f,%f,%f\n", blackbox.gyro_x_dps, blackbox.gyro_y_dps, blackbox.gyro_z_dps);
        return 1;
    }
    else 
    {
        // Okunacak veri kalmadı. Dosyayı kapat
        printf(header);
        blackbox_close_bin_file();
        return 0;
    }
}

// "blackbox.bin" dosyası oluştur. Binary veri tipinde yazmak için aç. Dosya zaten varsa üzerine yaz.
static void blackbox_create_open_to_write_bin_file()
{
    ESP_LOGI(TAG, "Opening file for binary writing");
    file_handler = fopen("/spiffs/blackbox.bin", "wb");
    if (file_handler == NULL) 
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
    }
}

// "blackbox.bin" dosyasını binary okumak için aç ve 1 döndür. Dosya yoksa veya açılamadıysa 0 döndür.
static uint8_t blackbox_open_to_read_bin_file()
{
    // Dosyayı binary modda aç ve verileri oku
    ESP_LOGI(TAG, "Opening file for binary reading");
    file_handler = fopen("/spiffs/blackbox.bin", "rb");
    if (file_handler == NULL) 
    {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return 0;
    }
    return 1;
}

// "blackbox.bin" dosyasını kapat
static void blackbox_close_bin_file()
{
    if (file_handler != NULL)
    {
        fclose(file_handler);
        file_handler = NULL;
        ESP_LOGI(TAG, "File closed");
    }
}

// "blackbox.bin" dosyasına "blackbox" yapısını kaydet
static void write_data_to_bin_file()
{
    if (file_handler != NULL)
    {
        blackbox.gyro_x_dps = imu_ptr->gyro_dps[X];
        blackbox.gyro_y_dps = imu_ptr->gyro_dps[Y];
        blackbox.gyro_z_dps = imu_ptr->gyro_dps[Z];
        fwrite(&blackbox, sizeof(blackbox_t), 1, file_handler);
    }
}

