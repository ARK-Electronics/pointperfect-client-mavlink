#include "UbxFrameScanner.hpp"

void UbxFrameScanner::feed(const uint8_t* data, size_t length, const UbxHandler& on_ubx, const RawHandler& on_raw)
{
	_buffer.insert(_buffer.end(), data, data + length);

	const size_t size = _buffer.size();
	size_t pos = 0;       // scan cursor
	size_t raw_start = 0; // start of the pending raw span

	while (pos < size) {
		if (_buffer[pos] != kSync1) {
			pos++;
			continue;
		}

		const size_t available = size - pos;

		// Sync byte at the tail: wait for enough data to decide.
		if (available < 2) {
			break;
		}

		if (_buffer[pos + 1] != kSync2) {
			pos++;
			continue;
		}

		if (available < 6) {
			break; // length field not complete yet
		}

		if (_buffer[pos + 2] != kMgaClass) {
			pos++; // not assistance data; leave it in the passthrough stream
			continue;
		}

		const size_t payload_length = _buffer[pos + 4] | (size_t(_buffer[pos + 5]) << 8);

		if (payload_length > kMaxPayloadLength) {
			pos++; // sync bytes inside correction data, not a frame
			continue;
		}

		const size_t frame_length = kFrameOverhead + payload_length;

		if (available < frame_length) {
			break; // frame not complete yet
		}

		// UBX checksum: 8-bit Fletcher over class..payload
		uint8_t ck_a = 0;
		uint8_t ck_b = 0;

		for (size_t i = pos + 2; i < pos + frame_length - 2; i++) {
			ck_a += _buffer[i];
			ck_b += ck_a;
		}

		if (ck_a != _buffer[pos + frame_length - 2] || ck_b != _buffer[pos + frame_length - 1]) {
			pos++;
			continue;
		}

		if (pos > raw_start) {
			on_raw(&_buffer[raw_start], pos - raw_start);
		}

		on_ubx(&_buffer[pos], frame_length);
		pos += frame_length;
		raw_start = pos;
	}

	// Everything before the cursor is settled raw data; bytes from the cursor
	// on could still become a valid frame and stay buffered.
	if (pos > raw_start) {
		on_raw(&_buffer[raw_start], pos - raw_start);
	}

	_buffer.erase(_buffer.begin(), _buffer.begin() + pos);
}

void UbxFrameScanner::flush(const RawHandler& on_raw)
{
	if (!_buffer.empty()) {
		on_raw(_buffer.data(), _buffer.size());
		_buffer.clear();
	}
}

void UbxFrameScanner::reset()
{
	_buffer.clear();
}
