
#include <Notecard.h>
#include <HardwareSerial.h>
#include "driver/uart.h"

#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
#include "camera_pins.h"

#include "esp_camera.h"

// Project
#define	PRODUCT_UID "com.blues.camnote"

// Camera framebuffer count. Used both when initializing the esp32-camera
// driver and by the stale-frame drain loop in loop().
#define CAMERA_FB_COUNT 2

// UART configuration
//#define UART_DEFAULT_SPEED  115200
//#define UART_DEFAULT_SPEED  460800
#define UART_DEFAULT_SPEED  921600
#define UART_SPEED          UART_DEFAULT_SPEED

// Pin assignments
#define PIN_TX   D6   // GPIO43
#define PIN_RX   D7   // GPIO44

// Debug goes to USB CDC, notecard uses UART0 (native on GPIO43/44)
#define debug Serial
#define notecardSerial Serial0
Notecard notecard;

int imageCount = 1;
bool cameraReady = false;
const char *notecardErr = NULL;

// Rolling frames-per-minute counter: 12 buckets of 5 seconds each. The
// bucket index is derived from (millis() / FPM_BUCKET_MS) % FPM_BUCKETS,
// and fpmLastBucket tracks the absolute bucket number we last wrote to so
// that we can lazily zero out any buckets we've skipped over since then.
#define FPM_BUCKETS    12
#define FPM_BUCKET_MS  5000
static uint16_t fpmBucket[FPM_BUCKETS] = {0};
static uint32_t fpmLastBucket = 0;

// Record a captured frame into the rolling FPM window and return the total
// number of frames observed across the most recent FPM_BUCKETS * FPM_BUCKET_MS
// (= 60s) of wall-clock time.
uint32_t fpmRecordAndSum()
{
	uint32_t nowBucket = millis() / FPM_BUCKET_MS;
	uint32_t advance = nowBucket - fpmLastBucket;
	if (advance >= FPM_BUCKETS) {
		// More than a full window has passed — everything older is stale.
		memset(fpmBucket, 0, sizeof(fpmBucket));
	} else {
		// Zero out any buckets we skipped over while idle.
		for (uint32_t i = 1; i <= advance; i++) {
			fpmBucket[(fpmLastBucket + i) % FPM_BUCKETS] = 0;
		}
	}
	fpmLastBucket = nowBucket;
	fpmBucket[nowBucket % FPM_BUCKETS]++;

	uint32_t sum = 0;
	for (uint32_t i = 0; i < FPM_BUCKETS; i++) {
		sum += fpmBucket[i];
	}
	return sum;
}

void setup()
{

	// Initialize debug
    debug.begin(115200);
    for (uint32_t expires = millis() + 2500; !debug && millis() < expires;);
	debug.printf("*** camnote ***\n");

	// Initialize interface to Notecard
#ifdef DEBUG_NOTECARD
    notecard.setDebugOutputStream(debug);
#endif
    notecard.begin(notecardSerial, UART_SPEED);

	// Verify connectivity to the notecard at that speed
	if (!notecard.ping()) {
		notecardErr = "Failed to ping Notecard over UART";
		return;
	}

    // Initialize the Notecard's I2C port
    J *req = notecard.newRequest("hub.set");
    JAddStringToObject(req, "product", PRODUCT_UID);
    JAddStringToObject(req, "mode", "continuous");
	JAddBoolToObject(req, "unsecure", true);
    if (!notecard.sendRequest(req)) {
		notecardErr = "Failed to set Notecard hub mode";
		return;
	}

	// Initialize camera
	camera_config_t config;
	config.ledc_channel = LEDC_CHANNEL_0;
	config.ledc_timer = LEDC_TIMER_0;
	config.pin_d0 = Y2_GPIO_NUM;
	config.pin_d1 = Y3_GPIO_NUM;
	config.pin_d2 = Y4_GPIO_NUM;
	config.pin_d3 = Y5_GPIO_NUM;
	config.pin_d4 = Y6_GPIO_NUM;
	config.pin_d5 = Y7_GPIO_NUM;
	config.pin_d6 = Y8_GPIO_NUM;
	config.pin_d7 = Y9_GPIO_NUM;
	config.pin_xclk = XCLK_GPIO_NUM;
	config.pin_pclk = PCLK_GPIO_NUM;
	config.pin_vsync = VSYNC_GPIO_NUM;
	config.pin_href = HREF_GPIO_NUM;
	config.pin_sccb_sda = SIOD_GPIO_NUM;
	config.pin_sccb_scl = SIOC_GPIO_NUM;
	config.pin_pwdn = PWDN_GPIO_NUM;
	config.pin_reset = RESET_GPIO_NUM;
	config.xclk_freq_hz = 20000000;
	config.pixel_format = PIXFORMAT_JPEG;
	config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
	config.fb_location = CAMERA_FB_IN_PSRAM;
	config.fb_count = 1;
	config.fb_count = CAMERA_FB_COUNT;
	config.grab_mode = CAMERA_GRAB_LATEST;

	config.jpeg_quality = 12;					// 10-15 is reasonable
	config.frame_size = FRAMESIZE_HD;			// UXGA:1600x1200, HD:1280x720, SVGA:800x600, VGA:640x480

	esp_err_t err = esp_camera_init(&config);
	if (err != ESP_OK) {
		Serial.printf("Camera init failed with error 0x%x", err);
		return;
	}

	cameraReady = true;

}

void loop() {

	if (notecardErr != NULL) {
		debug.println(notecardErr);
		delay(2500);
	}

	if (cameraReady) {

		// Don't bother capturing or uploading anything until the Notecard has
		// acquired time. Without a valid epoch we can't build a unique
		// filename, and the web.post would fail downstream anyway.
		if (!NoteTimeValidST()) {
			debug.println("Waiting for Notecard time to become valid...");
			delay(2500);
			return;
		}

		// Drain any stale frames the DMA buffered up while we were busy
		// uploading the previous image. With fb_count=2 + GRAB_LATEST the
		// driver can hold up to two already-completed frames in its queue,
		// and fb_get() returns the oldest of them. Popping fb_count frames
		// and discarding them guarantees the next capture represents a
		// moment at or after this loop iteration, not an arbitrarily stale
		// frame from seconds ago.
		for (int i = 0; i < CAMERA_FB_COUNT; i++) {
			camera_fb_t *stale = esp_camera_fb_get();
			if (!stale) break;
			esp_camera_fb_return(stale);
		}

		// Take a photo
		camera_fb_t *fb = esp_camera_fb_get();
		if (!fb) {
			Serial.println("Failed to get camera frame buffer");
			return;
		}

		// Allocate a PSRAM buffer large enough for the in-place COBS encode.
		// NoteBinaryStoreTransmit encodes in place and requires one extra byte
		// for the trailing newline packet terminator.
		uint32_t bufLen = NoteBinaryCodecMaxEncodedLength(fb->len) + 1;
		uint8_t *buf = (uint8_t *) ps_malloc(bufLen);
		if (buf == NULL) {
			debug.println("Failed to allocate binary transmit buffer");
			esp_camera_fb_return(fb);
			return;
		}

		// Copy the JPEG out of the camera frame buffer so we can return the
		// frame buffer to the driver, then COBS-encode and transmit it to the
		// Notecard's large binary store at offset 0 (single-shot transfer).
		memcpy(buf, fb->buf, fb->len);
		uint32_t jpegLen = fb->len;
		esp_camera_fb_return(fb);

		uint32_t xferStartMs = millis();
		const char *err = NoteBinaryStoreTransmit(buf, jpegLen, bufLen, 0);
		uint32_t xferElapsedMs = millis() - xferStartMs;
		free(buf);
		if (err != NULL) {
			debug.printf("NoteBinaryStoreTransmit failed: %s\n", err);
			return;
		}

		// Issue a web.post that consumes the staged binary buffer and routes
		// it through the "photos" route. The "name" field is used as the
		// filename/subpath appended to the route's base URL. Use the
		// time-suppressed Notecard epoch seconds plus the sub-second portion
		// of the monotonic millis() clock to keep filenames unique.
		char filename[40];
		JTIME epochSecs = NoteTimeST();
		uint32_t subSecMs = (uint32_t) (millis() % 1000);
		snprintf(filename, sizeof(filename), "img-%lu%03lu.jpg",
				 (unsigned long) epochSecs, (unsigned long) subSecMs);

		// Issue the web.post as a request with async:true. We still wait
		// for the response and time around requestAndResponse to get an
		// accurate to-notehub rate for this frame.
		J *req = notecard.newRequest("web.post");
		if (req != NULL) {
			JAddStringToObject(req, "route", "photos");
			JAddStringToObject(req, "name", filename);
			JAddStringToObject(req, "content", "image/jpeg");
			JAddBoolToObject(req, "binary", true);
			JAddBoolToObject(req, "async", true);
			uint32_t postStartMs = millis();
			J *rsp = notecard.requestAndResponse(req);
			uint32_t postElapsedMs = millis() - postStartMs;
			if (rsp == NULL) {
				debug.println("web.post: no response from Notecard");
			} else {
				const char *postErr = JGetString(rsp, "err");
				if (postErr != NULL && postErr[0] != '\0') {
					debug.printf("web.post failed: %s\n", postErr);
				} else {
					// Bytes/sec = payloadBytes * 1000 / elapsedMs. 64-bit
					// intermediate avoids overflow on large frames, and we
					// guard against a sub-millisecond elapsed time.
					uint32_t xferBps = xferElapsedMs ? (uint32_t)((uint64_t)jpegLen * 1000 / xferElapsedMs) : 0;
					uint32_t postBps = postElapsedMs ? (uint32_t)((uint64_t)jpegLen * 1000 / postElapsedMs) : 0;
					uint32_t fpm = fpmRecordAndSum();
					debug.printf("*** %s (%u bytes): to-notecard %u B/s, to-notehub %u B/s, %u fpm ***\n",
								 filename, (unsigned) jpegLen,
								 (unsigned) xferBps, (unsigned) postBps, (unsigned) fpm);
				}
				notecard.deleteResponse(rsp);
			}
		}

		// Done
		imageCount++;
	}
}
