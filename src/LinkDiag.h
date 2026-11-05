#pragma once

/**
 * @file LinkDiag.h
 * @brief Link diagnostics firmware modes, selected with AUDIO_MODULE_DIAG_MODE (see LinkDiag.cpp).
 */

#ifndef AUDIO_MODULE_DIAG_MODE
#define AUDIO_MODULE_DIAG_MODE 0
#endif

#ifndef AUDIO_MODULE_BAUD
#define AUDIO_MODULE_BAUD 500000
#endif

#if AUDIO_MODULE_DIAG_MODE != 0
void linkDiagSetup();
void linkDiagRun();
#endif
