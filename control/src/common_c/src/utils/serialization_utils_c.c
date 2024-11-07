//
#include "utils/serialization_utils_c.h"

#include "utils/com_utils_c.h"
#include "utils/log_c.h"


/// Uint8_t
Int32_t Phoenix_Common_GetSizeOfEncodedUint8Array(
    const Uint8_t* p, Int32_t elements) {
  (void) p;
  return (sizeof(Uint8_t) * elements);
}

Int32_t Phoenix_Common_EncodeUint8Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Uint8_t* p, Int32_t elements) {
  if (maxlen < elements) {
    LOG_ERR_C("Failed to encode \"Uint8_t\", (maxlen=%d)<(elements=%d)",
              maxlen, elements);
    return -1;
  }

  Uint8_t* buffer = (Uint8_t*) buf;
  phoenix_com_memcpy(&buffer[offset], p, elements);

  return (elements);
}

Int32_t Phoenix_Common_DecodeUint8Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Uint8_t* p, Int32_t elements) {
  if (maxlen < elements) {
    LOG_ERR_C("Failed to decode \"Uint8_t\", (maxlen=%d)<(elements=%d)",
              maxlen, elements);
    return -1;
  }

  const Uint8_t* buffer = (const Uint8_t*) buf;
  phoenix_com_memcpy(p, &buffer[offset], elements);

  return (elements);
}

void Phoenix_Common_CloneUint8Array(
    const Uint8_t* p, Uint8_t* q, Int32_t elements) {
  phoenix_com_memcpy(q, p, elements * sizeof(Uint8_t));
}


/// Int8_t
Int32_t Phoenix_Common_GetSizeOfEncodedInt8Array(
    const Int8_t* p, Int32_t elements) {
  (void) p;
  return (sizeof(Int8_t) * elements);
}

Int32_t Phoenix_Common_EncodeInt8Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Int8_t* p, Int32_t elements) {
  if (maxlen < elements) {
    LOG_ERR_C("Failed to encode \"Int8_t\", (maxlen=%d)<(elements=%d)",
              maxlen, elements);
    return -1;
  }

  Int8_t* buffer = (Int8_t*) buf;
  phoenix_com_memcpy(&buffer[offset], p, elements);

  return (elements);
}

Int32_t Phoenix_Common_DecodeInt8Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Int8_t* p, Int32_t elements) {
  if (maxlen < elements) {
    LOG_ERR_C("Failed to decode \"Int8_t\", (maxlen=%d)<(elements=%d)",
              maxlen, elements);
    return -1;
  }

  const Int8_t* buffer = (const Int8_t*) buf;
  phoenix_com_memcpy(p, &buffer[offset], elements);

  return (elements);
}

void Phoenix_Common_CloneInt8Array(
    const Int8_t* p, Int8_t* q, Int32_t elements) {
  phoenix_com_memcpy(q, p, elements * sizeof(Int8_t));
}


/// Int16_t
Int32_t Phoenix_Common_GetSizeOfEncodedInt16Array(
    const Int16_t* p, Int32_t elements) {
  (void) p;
  return (sizeof(Int16_t) * elements);
}

Int32_t Phoenix_Common_EncodeInt16Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Int16_t* p, Int32_t elements) {
  Int32_t total_size = sizeof(Int16_t) * elements;
  Uint8_t* buffer = (Uint8_t*) buf;
  Int32_t pos = offset;
  Int32_t index = 0;

  if (maxlen < total_size) {
    LOG_ERR_C("Failed to encode \"Int16_t\", (maxlen=%d)<(elements=%d)",
              maxlen, elements);
    return -1;
  }

  //  See Section 5.8 paragraph 3 of the standard
  //  http://open-std.org/JTC1/SC22/WG21/docs/papers/2015/n4527.pdf
  //  use uint for shifting instead if int
  const Uint16_t* unsigned_p = (const Uint16_t*) p;
  for (index = 0; index < elements; index++) {
    Uint16_t v = unsigned_p[index];
    buffer[pos++] = (v >> 8) & 0xff;
    buffer[pos++] = (v & 0xff);
  }

  return (total_size);
}

Int32_t Phoenix_Common_DecodeInt16Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Int16_t* p, Int32_t elements) {
  Int32_t total_size = sizeof(Int16_t) * elements;
  const Uint8_t* buffer = (const Uint8_t*) buf;
  Int32_t pos = offset;
  Int32_t index = 0;

  if (maxlen < total_size) {
    LOG_ERR_C("Failed to decode \"Int16_t\", (maxlen=%d)<(total_size=%d)",
              maxlen, total_size);
    return -1;
  }

  for (index = 0; index < elements; index++) {
    p[index] = (buffer[pos] << 8) + buffer[pos + 1];
    pos += 2;
  }

  return (total_size);
}

void Phoenix_Common_CloneInt16Array(
    const Int16_t* p, Int16_t* q, Int32_t elements) {
  phoenix_com_memcpy(q, p, elements * sizeof(Int16_t));
}


/// Uint16_t
Int32_t Phoenix_Common_GetSizeOfEncodedUint16Array(
    const Uint16_t* p, Int32_t elements) {
  (void) p;
  return (sizeof(Uint16_t) * elements);
}

Int32_t Phoenix_Common_EncodeUint16Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Uint16_t* p, Int32_t elements) {
  return Phoenix_Common_EncodeInt16Array(
        buf, offset, maxlen, (const Int16_t*) p, elements);
}

Int32_t Phoenix_Common_DecodeUint16Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Uint16_t* p, Int32_t elements) {
  return Phoenix_Common_DecodeInt16Array(
        buf, offset, maxlen, (Int16_t*) p, elements);
}

void Phoenix_Common_CloneUint16Array(
    const Uint16_t* p, Uint16_t* q, Int32_t elements) {
  phoenix_com_memcpy(q, p, elements * sizeof(Uint16_t));
}


/// Int32_t
Int32_t Phoenix_Common_GetSizeOfEncodedInt32Array(
    const Int32_t* p, Int32_t elements) {
  (void) p;
  return (sizeof(Int32_t) * elements);
}

Int32_t Phoenix_Common_EncodeInt32Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Int32_t* p, Int32_t elements) {
  Int32_t total_size = sizeof(Int32_t) * elements;
  Uint8_t* buffer = (Uint8_t *) buf;
  Int32_t pos = offset;
  Int32_t index = 0;

  if (maxlen < total_size) {
    LOG_ERR_C("Failed to encode \"Int32_t\", (maxlen=%d)<(total_size=%d)",
              maxlen, total_size);
    return -1;
  }

  //  See Section 5.8 paragraph 3 of the standard
  //  http://open-std.org/JTC1/SC22/WG21/docs/papers/2015/n4527.pdf
  //  use uint for shifting instead if int
  const Uint32_t* unsigned_p = (const Uint32_t*) p;
  for (index = 0; index < elements; index++) {
    const Uint32_t v = unsigned_p[index];
    buffer[pos++] = (v >> 24) & 0xff;
    buffer[pos++] = (v >> 16) & 0xff;
    buffer[pos++] = (v >> 8) & 0xff;
    buffer[pos++] = (v & 0xff);
  }

  return (total_size);
}

Int32_t Phoenix_Common_DecodeInt32Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Int32_t* p, Int32_t elements) {
  Int32_t total_size = sizeof(Int32_t) * elements;
  const Uint8_t* buffer = (const Uint8_t*) buf;
  Int32_t pos = offset;
  Int32_t index = 0;

  if (maxlen < total_size) {
    LOG_ERR_C("Failed to decode \"Int32_t\", (maxlen=%d)<(total_size=%d)",
              maxlen, total_size);
    return -1;
  }

  //  See Section 5.8 paragraph 3 of the standard
  //  http://open-std.org/JTC1/SC22/WG21/docs/papers/2015/n4527.pdf
  //  use uint for shifting instead if int
  for (index = 0; index < elements; index++) {
    p[index] =
        (((Uint32_t) buffer[pos + 0]) << 24) +
        (((Uint32_t) buffer[pos + 1]) << 16) +
        (((Uint32_t) buffer[pos + 2]) << 8) +
        ((Uint32_t)  buffer[pos + 3]);
    pos += 4;
  }

  return (total_size);
}

void Phoenix_Common_CloneInt32Array(
    const Int32_t* p, Int32_t* q, Int32_t elements) {
  phoenix_com_memcpy(q, p, elements * sizeof(Int32_t));
}


/// Uint32_t
Int32_t Phoenix_Common_GetSizeOfEncodedUint32Array(
    const Uint32_t* p, Int32_t elements) {
  (void) p;
  return (sizeof(Uint32_t) * elements);
}

Int32_t Phoenix_Common_EncodeUint32Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Uint32_t* p, Int32_t elements) {
  return Phoenix_Common_EncodeInt32Array(
        buf, offset, maxlen, (const Int32_t*) p, elements);
}

Int32_t Phoenix_Common_DecodeUint32Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Uint32_t* p, Int32_t elements) {
  return Phoenix_Common_DecodeInt32Array(
        buf, offset, maxlen, (Int32_t*) p, elements);
}

void Phoenix_Common_CloneUint32Array(
    const Uint32_t* p, Uint32_t* q, Int32_t elements) {
  phoenix_com_memcpy(q, p, elements * sizeof(Uint32_t));
}


/// Int64_t
Int32_t Phoenix_Common_GetSizeOfEncodedInt64Array(
    const Int64_t* p, Int32_t elements) {
  (void) p;
  return (sizeof(Int64_t) * elements);
}

Int32_t Phoenix_Common_EncodeInt64Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Int64_t* p, Int32_t elements) {
  Int32_t total_size = sizeof(Int64_t) * elements;
  Uint8_t* buffer = (Uint8_t*) buf;
  Int32_t pos = offset;
  Int32_t index = 0;

  if (maxlen < total_size) {
    LOG_ERR_C("Failed to encode \"Int64_t\", (maxlen=%d)<(total_size=%d)",
              maxlen, total_size);
    return -1;
  }

  //  See Section 5.8 paragraph 3 of the standard
  //  http://open-std.org/JTC1/SC22/WG21/docs/papers/2015/n4527.pdf
  //  use uint for shifting instead if int
  const Uint64_t* unsigned_p = (const Uint64_t*) p;
  for (index = 0; index < elements; index++) {
    const Uint64_t v = unsigned_p[index];
    buffer[pos++] = (v >> 56) & 0xff;
    buffer[pos++] = (v >> 48) & 0xff;
    buffer[pos++] = (v >> 40) & 0xff;
    buffer[pos++] = (v >> 32) & 0xff;
    buffer[pos++] = (v >> 24) & 0xff;
    buffer[pos++] = (v >> 16) & 0xff;
    buffer[pos++] = (v >> 8) & 0xff;
    buffer[pos++] = (v & 0xff);
  }

  return (total_size);
}

Int32_t Phoenix_Common_DecodeInt64Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Int64_t* p, Int32_t elements) {
  Int32_t total_size = sizeof(Int64_t) * elements;
  const Uint8_t* buffer = (const Uint8_t *) buf;
  Int32_t pos = offset;
  Int32_t index = 0;

  if (maxlen < total_size) {
    LOG_ERR_C("Failed to decode \"Int64_t\", (maxlen=%d)<(total_size=%d)",
              maxlen, total_size);
    return -1;
  }

  //  See Section 5.8 paragraph 3 of the standard
  //  http://open-std.org/JTC1/SC22/WG21/docs/papers/2015/n4527.pdf
  //  use uint for shifting instead if int
  for (index = 0; index < elements; index++) {
    Uint64_t a =
        (((Uint32_t) buffer[pos + 0]) << 24) +
        (((Uint32_t) buffer[pos + 1]) << 16) +
        (((Uint32_t) buffer[pos + 2]) << 8) +
        ((Uint32_t)  buffer[pos + 3]);
    pos += 4;
    Uint64_t b =
        (((Uint32_t) buffer[pos + 0]) << 24) +
        (((Uint32_t) buffer[pos + 1]) << 16) +
        (((Uint32_t) buffer[pos + 2]) << 8) +
        ((Uint32_t)  buffer[pos + 3]);
    pos += 4;
    p[index] = (a << 32) + (b & 0xffffffff);
  }

  return total_size;
}

void Phoenix_Common_CloneInt64Array(
    const Int64_t* p, Int64_t* q, Int32_t elements) {
  phoenix_com_memcpy(q, p, elements * sizeof(Int64_t));
}


/// Uint64_t
Int32_t Phoenix_Common_GetSizeOfEncodedUint64Array(
    const Uint64_t* p, Int32_t elements) {
  (void) p;
  return (sizeof(Uint64_t) * elements);
}

Int32_t Phoenix_Common_EncodeUint64Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Uint64_t* p, Int32_t elements) {
  return Phoenix_Common_EncodeInt64Array(
        buf, offset, maxlen, (const Int64_t*) p, elements);
}

Int32_t Phoenix_Common_DecodeUint64Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Uint64_t* p, Int32_t elements) {
  return Phoenix_Common_DecodeInt64Array(
        buf, offset, maxlen, (Int64_t*) p, elements);
}

void Phoenix_Common_CloneUint64Array(
    const Uint64_t* p, Uint64_t* q, Int32_t elements) {
  phoenix_com_memcpy(q, p, elements * sizeof(Uint64_t));
}


/// Float32_t
Int32_t Phoenix_Common_GetSizeOfEncodedFloat32Array(
    const Float32_t* p, Int32_t elements) {
  (void) p;
  return (sizeof(Float32_t) * elements);
}

Int32_t Phoenix_Common_EncodeFloat32Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Float32_t* p, Int32_t elements) {
  return Phoenix_Common_EncodeInt32Array(
        buf, offset, maxlen, (const Int32_t*) p, elements);
}

Int32_t Phoenix_Common_DecodeFloat32Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Float32_t* p, Int32_t elements) {
  return Phoenix_Common_DecodeInt32Array(
        buf, offset, maxlen, (Int32_t*) p, elements);
}

void Phoenix_Common_CloneFloat32Array(
    const Float32_t* p, Float32_t* q, Int32_t elements) {
  phoenix_com_memcpy(q, p, elements * sizeof(Float32_t));
}


/// Float64_t
Int32_t Phoenix_Common_GetSizeOfEncodedFloat64Array(
    const Float64_t *p, Int32_t elements) {
  (void) p;
  return (sizeof(Float64_t) * elements);
}

Int32_t Phoenix_Common_EncodeFloat64Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Float64_t* p, Int32_t elements) {
  return Phoenix_Common_EncodeInt64Array(
        buf, offset, maxlen, (const Int64_t*) p, elements);
}

Int32_t Phoenix_Common_DecodeFloat64Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Float64_t* p, Int32_t elements) {
  return Phoenix_Common_DecodeInt64Array(
        buf, offset, maxlen, (Int64_t*) p, elements);
}

void Phoenix_Common_CloneFloat64Array(
    const Float64_t* p, Float64_t* q, Int32_t elements) {
  phoenix_com_memcpy(q, p, elements * sizeof(Float64_t));
}

