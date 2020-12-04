/*
* Copyright 2019 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/**
 * @file
 *   This file is a wrapper for the standard "string.h" file
 *   Some platforms provide all required functions, some do not.
 *   This solves the missing functions in #include <string.h>
 */

#if !defined(WRAP_STRING_H)
#define WRAP_STRING_H

/* system provided string.h */
#include <string.h>

/* Prototypes for our missing function replacements */
size_t missing_strlcpy(char *dst, const char *src, size_t dstsize);

#endif // WRAP_STRING_H
