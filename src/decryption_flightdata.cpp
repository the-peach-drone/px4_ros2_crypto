#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
// #include <tss2/tss2_esys.h>
#include <tss2/tss2_fapi.h>
#include <cstring>

typedef struct {
	uint64_t timestamp;
	float	gyro[3];
} SensorData;

void ConvertUint8ArrayToData(const uint8_t *data, SensorData *result)
{
    memcpy(&result->timestamp, data, sizeof(uint64_t));

    for (int i = 0; i < 3; i++) {
        memcpy(&result->gyro[i], data + sizeof(uint64_t) + i * sizeof(float), sizeof(float));
    }
}

int main(int argc, char *argv[])
{
	std::cout << "Starting decrypt..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	
    FILE *file = fopen("data.txt", "r");
    fseek(file, 0, SEEK_END);
    long fileLength = ftell(file);
    fseek(file, 0, SEEK_SET);

    uint8_t *cipher_text = (uint8_t*)malloc(fileLength / 2);

    char hexPair[3];
    hexPair[2] = '\0';

    for (long i = 0; i < fileLength/2;i++) {
        fread(hexPair, 1, 2, file);
        cipher_text[i] = (uint8_t)strtol(hexPair, NULL, 16);
    }
    fclose(file);

    FAPI_CONTEXT *context = NULL;
    Fapi_Initialize(&context, NULL);

    uint8_t *plain_text = NULL;
    size_t plain_text_size = 0;

    SensorData sensor_data;
    
    for (long i = 0; i < fileLength/2; i+=256) {    
        Fapi_Decrypt(context, "/HS/SRK/rsa_key", cipher_text + i, 256, &plain_text, &plain_text_size);
        ConvertUint8ArrayToData(plain_text, &sensor_data);

        std::cout << "\nDecrypt SENSOR COMBINED DATA"   << std::endl;
        std::cout << "============================="   << std::endl;
        std::cout << "ts: " << sensor_data.timestamp << std::endl;
        std::cout << "gyro[0]: " << sensor_data.gyro[0] << std::endl;
        std::cout << "gyro[1]: " << sensor_data.gyro[1] << std::endl;
        std::cout << "gyro[2]: " << sensor_data.gyro[2] << std::endl;
        std::cout << std::endl;
    }
	rclcpp::shutdown();
	return 0;
}