#ifndef SM_EVENTS_HPP
#define SM_EVENTS_HPP

#include <stdint.h>

/* Eventos que la GUI / CAN postean y la máquina de estados consume. */
enum class SmEvent : uint8_t {
    NONE   = 0,
    MOVE,    /* movimiento de joint solicitado */
    STOP,    /* seta / parada de emergencia */
    PAUSE,
    RESUME
};

/* Publicar evento (thread-safe). Web handlers / callbacks CAN. */
void sm_post(SmEvent ev);
/* True si el evento está pendiente; lo limpia. Usado por la SM. */
bool sm_consume(SmEvent ev);
/* Estado global de "sistema listo" (INIT -> IDLE). */
void sm_setReady(bool ready);
bool sm_isReady();
void sm_clearAll();

#endif