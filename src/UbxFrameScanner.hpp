#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

// Splits a GNSS correction byte stream into complete UBX frames and
// passthrough spans of everything else (RTCM3/SPARTN), preserving stream
// order. The PointPerfect *-MGA mountpoints prepend UBX-MGA assistance
// messages to the correction stream; those need to be re-emitted
// message-aligned, while correction bytes stay an opaque passthrough.
class UbxFrameScanner
{
public:
	// Receives one complete UBX frame (sync bytes through checksum).
	using UbxHandler = std::function<void(const uint8_t* frame, size_t length)>;
	// Receives non-UBX bytes, in stream order relative to UBX frames.
	using RawHandler = std::function<void(const uint8_t* data, size_t length)>;

	// Consume a stream chunk, emitting complete UBX frames and the raw bytes
	// around them. Bytes that could still become a valid frame stay buffered.
	void feed(const uint8_t* data, size_t length, const UbxHandler& on_ubx, const RawHandler& on_raw);

	// Emit everything still buffered as raw. Call when the stream goes idle
	// so a partial frame candidate cannot hold bytes back indefinitely.
	void flush(const RawHandler& on_raw);

	// Drop any buffered bytes (stream restarted).
	void reset();

	size_t buffered() const { return _buffer.size(); }

private:
	// UBX-MGA messages are well under this; a larger length means the sync
	// bytes were correction payload. Also bounds how long bytes can wait.
	static constexpr size_t kMaxPayloadLength = 512;
	static constexpr size_t kFrameOverhead = 8; // sync(2) class id length(2) checksum(2)
	static constexpr uint8_t kSync1 = 0xB5;
	static constexpr uint8_t kSync2 = 0x62;

	std::vector<uint8_t> _buffer;
};
