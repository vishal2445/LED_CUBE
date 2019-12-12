// Custom data types

// An RGB value as understood by our hardware
typedef struct _RGB {
  uint16_t red;
  uint16_t green;
  uint16_t blue;
} RGB;

// A 2D point in our cube space
typedef struct _POINT2D {
  uint8_t x;
  uint8_t y;
} POINT2D;

// A 3D point in our cube space
typedef struct _POINT3D {
  uint8_t x;
  uint8_t y;
  uint8_t z;
} POINT3D;

// A function pointer for executing display patterns
typedef void (*pt2Function)();

