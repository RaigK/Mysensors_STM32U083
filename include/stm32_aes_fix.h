/**
 * STM32 MySensors Compatibility Fixes
 *
 * Fixes AES macro conflict between STM32 HAL and MySensors library.
 * This file must be force-included before all other headers.
 */

#ifndef STM32_AES_FIX_H
#define STM32_AES_FIX_H

// Will be applied after STM32 headers define AES
// The actual undef happens in the patched MySensors AES files

#endif // STM32_AES_FIX_H
