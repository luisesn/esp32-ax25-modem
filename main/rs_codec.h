#pragma once
/*
 * Reed-Solomon para IL2P (compatible con Dire Wolf / especificación IL2P).
 *
 * GF(2^8), polinomio primitivo 0x11D, primera raíz α^0 (fcr=0), α primitivo.
 * Códigos acortados: se calcula como si el bloque estuviera precedido de
 * ceros hasta 255 símbolos. nroots válido: 2, 4, 6, 8 o 16 (cualquiera ≤16).
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* parity: nroots bytes. */
void il2p_rs_encode(const uint8_t *data, int dlen, int nroots, uint8_t *parity);

/* block = datos (dlen) + paridad (nroots). Corrige en sitio hasta nroots/2
 * símbolos. Devuelve nº de símbolos corregidos o -1 si no es recuperable. */
int  il2p_rs_decode(uint8_t *block, int dlen, int nroots);

#ifdef __cplusplus
}
#endif
