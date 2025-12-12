#include <ArduinoBLE.h>
#include <PDM.h>
#include <speechML_inferencing.h>

#define UART_SERVICE_UUID   "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define UART_RX_UUID        "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define UART_TX_UUID        "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static const float kConfidenceThreshold = 0.80f;
static const unsigned long kScanRetryDelayMs = 1000;

BLEDevice peripheral;
BLECharacteristic rxChar;
BLECharacteristic txChar;

static signed short sampleBuffer[2048];
static bool debug_nn = false;

typedef struct {
    int16_t *buffer;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

static inference_t inference;
static uint32_t class_counts[EI_CLASSIFIER_LABEL_COUNT] = {0};
static uint32_t total_inferences = 0;
static uint32_t wav_counter = 0;

static bool microphone_inference_start(uint32_t n_samples);
static bool microphone_inference_record(void);
static void microphone_inference_end(void);
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr);
static void pdm_data_ready_inference_callback(void);
static void stream_audio_window(const int16_t *data, size_t length, uint32_t sample_rate);
static void print_ratio_table(void);
static void wait_for_enter_key(void);
static bool ensure_ble_connection(void);
static bool connect_to_device(BLEDevice device);
static void pump_ble_notifications(void);
static void send_command_over_ble(const char *label);
static bool send_command_payload(const char *command_keyword);
static void trigger_emergency_stop(void);
static void poll_emergency_shortcut(void);

void setup()
{
    Serial.begin(115200);
    while (!Serial) {}
    Serial.println("=== BLE Voice Controller - Seeed XIAO nRF52840 Sense ===");

    if (!BLE.begin()) {
        Serial.println("BLE khởi tạo thất bại!");
        while (true) {
            delay(1000);
        }
    }
    Serial.println("BLE đã sẵn sàng. Đang tìm Pico 2W...");

    if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {
        ei_printf("ERR: Could not allocate audio buffer (size %d)\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
        while (true) {
            delay(1000);
        }
    }

    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: %.2f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));
    ei_printf("\nHướng dẫn:\n");
    ei_printf("1) Thiết bị sẽ kết nối Pico 2W qua BLE (UART service).\n");
    ei_printf("2) Nhấn Enter trên Serial Monitor/PowerShell để ghi âm 1.5 s.\n");
    ei_printf("3) Nếu độ tin cậy >= %.0f%%, lệnh sẽ được gửi sang Pico.\n", kConfidenceThreshold * 100.0f);
}

void loop()
{
    poll_emergency_shortcut();

    if (!ensure_ble_connection()) {
        delay(kScanRetryDelayMs);
        return;
    }

    ei_printf("\n===== Sẵn sàng ghi 1.5 s =====\n");
    ei_printf("Nhấn Enter để bắt đầu ghi.\n");
    wait_for_enter_key();
    ei_printf("Chuẩn bị ghi âm...\n");
    delay(500);
    ei_printf("Đang ghi âm...\n");

    if (!microphone_inference_record()) {
        ei_printf("ERR: Failed to record audio...\n");
        return;
    }
    ei_printf("Đã ghi âm xong.\n");

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
    if (r != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", r);
        return;
    }

    size_t top_idx = 0;
    float top_score = 0.0f;
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        if (result.classification[ix].value > top_score) {
            top_score = result.classification[ix].value;
            top_idx = ix;
        }
    }

    class_counts[top_idx] += 1;
    total_inferences += 1;

    Serial.print("Kết quả dự đoán (DSP: ");
    Serial.print(result.timing.dsp);
    Serial.print(" ms, CLS: ");
    Serial.print(result.timing.classification);
    Serial.println(" ms):");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        Serial.print("    ");
        Serial.print(result.classification[ix].label);
        Serial.print(": ");
        Serial.print(result.classification[ix].value * 100.0f, 2);
        Serial.println(" %");
    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    Serial.print("    anomaly score: ");
    Serial.println(result.anomaly, 3);
#endif
    const char *top_label = result.classification[top_idx].label;
    Serial.println("===== Phân loại giọng nói =====");
    Serial.print("Lệnh mạnh nhất: ");
    Serial.print(top_label);
    Serial.print(" | Độ tin cậy: ");
    Serial.println(top_score, 3);

    if (top_score >= kConfidenceThreshold) {
        Serial.println("Độ tin cậy >= 0.80, chuẩn bị gửi BLE...");
        send_command_over_ble(top_label);
    } else {
        ei_printf("Độ tin cậy chưa đủ %.0f%%, không gửi lệnh.\n", kConfidenceThreshold * 100.0f);
        Serial.println("Độ tin cậy < 0.80, không gửi BLE.");
    }

    print_ratio_table();
}

static bool ensure_ble_connection(void)
{
    if (peripheral && peripheral.connected()) {
        pump_ble_notifications();
        return true;
    }

    Serial.println("Đang quét thiết bị BLE...");
    BLE.scanForUuid(UART_SERVICE_UUID);
    BLEDevice found = BLE.available();
    if (!found) {
        Serial.println("Chưa tìm thấy Pico 2W phù hợp.");
        return false;
    }
    BLE.stopScan();
    return connect_to_device(found);
}

static bool connect_to_device(BLEDevice device)
{
    Serial.print("Đang kết nối tới: ");
    Serial.println(device.address());
    if (!device.connect()) {
        Serial.println("Kết nối thất bại.");
        return false;
    }
    Serial.println("Kết nối thành công, đang đọc thuộc tính...");

    if (!device.discoverAttributes()) {
        Serial.println("Không đọc được thuộc tính!");
        device.disconnect();
        return false;
    }

    BLEService uartService = device.service(UART_SERVICE_UUID);
    if (!uartService) {
        Serial.println("Không tìm thấy UART Service!");
        device.disconnect();
        return false;
    }

    rxChar = uartService.characteristic(UART_RX_UUID);
    txChar = uartService.characteristic(UART_TX_UUID);

    if (!rxChar || !txChar) {
        Serial.println("Không tìm thấy đặc tính RX/TX!");
        device.disconnect();
        return false;
    }

    if (txChar.canSubscribe()) {
        txChar.subscribe();
    }

    peripheral = device;
    Serial.println("BLE đã sẵn sàng gửi/nhận lệnh giọng nói.");
    return true;
}

static void pump_ble_notifications(void)
{
    BLE.poll();
    if (peripheral && peripheral.connected() && txChar && txChar.valueUpdated()) {
        String value = String((const char *)txChar.value());
        Serial.print("Nhận từ Pico2W: ");
        Serial.println(value);
    }
}

static const char *map_label_to_command(const char *label)
{
    if (!label) {
        return NULL;
    }

    if (strcasecmp(label, "tiến lên") == 0 || strcasecmp(label, "tien len") == 0) {
        return "GO";
    }
    if (strcasecmp(label, "lùi lại") == 0 || strcasecmp(label, "lui lai") == 0) {
        return "BACK";
    }
    if (strcasecmp(label, "rẽ trái") == 0 || strcasecmp(label, "re trai") == 0) {
        return "LEFT";
    }
    if (strcasecmp(label, "rẽ phải") == 0 || strcasecmp(label, "re phai") == 0) {
        return "RIGHT";
    }
    if (strcasecmp(label, "dừng lại") == 0 || strcasecmp(label, "dung lai") == 0) {
        return "STOP";
    }
    return NULL;
}

static bool send_command_payload(const char *command_keyword)
{
    if (!peripheral || !peripheral.connected() || !rxChar) {
        ei_printf("Không thể gửi lệnh vì BLE chưa sẵn sàng.\n");
        return false;
    }

    unsigned long now_ms = millis();
    char payload[64];
    snprintf(payload, sizeof(payload), "%s", command_keyword, now_ms);

    if (rxChar.writeValue(payload)) {
        ei_printf("Đã gửi lệnh qua BLE: %s\n", payload, now_ms);
        return true;
    }

    ei_printf("Gửi BLE thất bại!\n");
    return false;
}

static void send_command_over_ble(const char *label)
{
    const char *command = map_label_to_command(label);
    if (!command) {
        ei_printf("Không nhận dạng được lệnh: %s\n", label ? label : "(null)");
        return;
    }
    send_command_payload(command);
}

static void trigger_emergency_stop(void)
{
    Serial.println(">>> KHẨN CẤP: phím SPACE được nhấn, gửi STOP!");
    if (!send_command_payload("STOP")) {
        Serial.println("Không thể gửi STOP khẩn cấp (BLE không sẵn sàng).");
    }
}

static void poll_emergency_shortcut(void)
{
    while (Serial.available()) {
        int next = Serial.peek();
        if (next < 0 || next != ' ') {
            break;
        }
        Serial.read();
        trigger_emergency_stop();
    }
}

static void wait_for_enter_key(void)
{
    while (Serial.available()) {
        Serial.read();
    }

    for (;;) {
        pump_ble_notifications();
        poll_emergency_shortcut();
        if (Serial.available()) {
            char c = Serial.read();
            if (c == ' ') {
                trigger_emergency_stop();
                continue;
            }
            if (c == '\n' || c == '\r') {
                while (Serial.available()) {
                    char next = Serial.peek();
                    if (next == '\n' || next == '\r') {
                        Serial.read();
                    }
                    else {
                        break;
                    }
                }
                break;
            }
        }
        delay(5);
    }
}

static void print_ratio_table(void)
{
    ei_printf("Tỷ lệ tích lũy:\n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        float ratio = (total_inferences == 0) ? 0.0f : ((float)class_counts[ix] / (float)total_inferences) * 100.0f;
        ei_printf("    %s: %lu lần (%.1f%%)\n", ei_classifier_inferencing_categories[ix], (unsigned long)class_counts[ix], ratio);
    }
}

static void stream_audio_window(const int16_t *data, size_t length, uint32_t sample_rate)
{
    char filename[24];
    snprintf(filename, sizeof(filename), "rec_%05lu.wav", (unsigned long)++wav_counter);
    ei_printf("WAV_BEGIN %s %lu %lu\n", filename, (unsigned long)sample_rate, (unsigned long)length);
    for (size_t i = 0; i < length; i++) {
        ei_printf("%d\n", data[i]);
    }
    ei_printf("WAV_END\n");
}

static void pdm_data_ready_inference_callback(void)
{
    int bytesAvailable = PDM.available();
    int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);

    if (inference.buf_ready == 0) {
        for (int i = 0; i < bytesRead >> 1; i++) {
            inference.buffer[inference.buf_count++] = sampleBuffer[i];
            if (inference.buf_count >= inference.n_samples) {
                inference.buf_count = 0;
                inference.buf_ready = 1;
                break;
            }
        }
    }
}

static bool microphone_inference_start(uint32_t n_samples)
{
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));
    if (inference.buffer == NULL) {
        return false;
    }
    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;

    PDM.onReceive(&pdm_data_ready_inference_callback);
    PDM.setBufferSize(4096);

    if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY)) {
        ei_printf("Failed to start PDM!\n");
        microphone_inference_end();
        return false;
    }

    PDM.setGain(50);
    return true;
}

static bool microphone_inference_record(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;
    while (inference.buf_ready == 0) {
        pump_ble_notifications();
        poll_emergency_shortcut();
        delay(10);
    }
    return true;
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);
    return 0;
}

static void microphone_inference_end(void)
{
    PDM.end();
    free(inference.buffer);
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif
