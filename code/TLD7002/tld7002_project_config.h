#ifndef TLD7002_PROJECT_CONFIG_H_
#define TLD7002_PROJECT_CONFIG_H_

// Set to 1 to let this project own UART1/P11.12/P11.10 for TLD7002.
#ifndef TLD7002_PROJECT_ENABLE
#define TLD7002_PROJECT_ENABLE              (1U)
#endif

// Set to 1 only when the TLD7002 dot-matrix board is connected.
#ifndef TLD7002_PROJECT_DOT_MATRIX_ENABLE
#define TLD7002_PROJECT_DOT_MATRIX_ENABLE   (1U)
#endif

#ifndef TLD7002_PROJECT_DEFAULT_BRIGHTNESS
#define TLD7002_PROJECT_DEFAULT_BRIGHTNESS  (5000U)
#endif

#endif /* TLD7002_PROJECT_CONFIG_H_ */
