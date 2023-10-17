/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *           2018 PX4 Pro Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Sensor Combined uORB topic listener example
 * @file sensor_combined_listener.cpp
 * @addtogroup examples
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 * @author Vicente Monge
 */
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

void ConvertDataToUint8Array(uint8_t *dest, SensorData data) {
	memcpy(dest, &data.timestamp, sizeof(uint64_t));

	for (int i = 0; i < 3; i++) {
		memcpy(dest + sizeof(uint64_t) + i * sizeof(float), &data.gyro[i], sizeof(float));
	}
}
/**
 * @brief Sensor Combined uORB topic data callback
 */
class SensorCombinedListener : public rclcpp::Node
{
public:
	explicit SensorCombinedListener() : Node("sensor_combined_listener")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		Fapi_Initialize(&context, NULL);
		file = fopen("data.txt", "w");
		// sizeFile = fopen("data_size.txt", "w");

		subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos,
		[this](const px4_msgs::msg::SensorCombined::UniquePtr msg) {
			std::cout << "\n\n\n\n\n";
			std::cout << "RAW SENSOR COMBINED DATA"   << std::endl;
			std::cout << "============================="   << std::endl;
			std::cout << "ts: "          << msg->timestamp    << std::endl;
			std::cout << "gyro_rad[0]: " << msg->gyro_rad[0]  << std::endl;
			std::cout << "gyro_rad[1]: " << msg->gyro_rad[1]  << std::endl;
			std::cout << "gyro_rad[2]: " << msg->gyro_rad[2]  << std::endl;
			
			sensor_data.timestamp = msg->timestamp;
			sensor_data.gyro[0] = msg->gyro_rad[0];
			sensor_data.gyro[1] = msg->gyro_rad[1];
			sensor_data.gyro[2] = msg->gyro_rad[2];

			ConvertDataToUint8Array(data_to_encrypt, sensor_data);
			Fapi_Encrypt(context, "/HS/SRK/rsa_key", &data_to_encrypt[0], sizeof(data_to_encrypt), &cipher, &cipher_size);

			std::cout << "\nEncrypt SENSOR COMBINED DATA"   << std::endl;
			std::cout << "============================="   << std::endl;
			// cipher text to textfile
			for (size_t i = 0; i < cipher_size; i++) {
				fprintf(file, "%02X", cipher[i]);
				printf("%02X", cipher[i]);
			}
			// cipher size to textfile
			// fprintf(sizeFile, "%zu", cipher_size);
		});
	}

private:
	rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscription_;

	SensorData sensor_data;
	uint8_t data_to_encrypt[sizeof(uint64_t) + 3 * sizeof(float)];

	FAPI_CONTEXT *context = NULL;
	
	uint8_t* cipher;
	size_t cipher_size;

	FILE *file;
	// FILE *sizeFile;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting sensor_combined listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SensorCombinedListener>());

	rclcpp::shutdown();
	return 0;
}
