#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H

#include <Arduino.h>
#include "config.h"


typedef enum {
    OBSTACLE_NONE,        // Engel yok
    OBSTACLE_FAR,         // Uzakta engel (yavaşla)
    OBSTACLE_CLOSE        // Yakında engel (dur)
} ObstacleLevel;

typedef struct {
    float         distance_cm[ULTRASONIC_COUNT]; // Ön, Sol-ön, Sağ-ön
    ObstacleLevel level;           // Genel engel seviyesi
    bool          front_blocked;   // Ön bloke
    bool          left_blocked;    // Sol-ön bloke
    bool          right_blocked;   // Sağ-ön bloke
} ObstacleData;

// --- API ---
void            obstacle_init(void);
void            obstacle_update(void);
ObstacleData*   obstacle_get_data(void);
bool            obstacle_is_blocked(void);
ObstacleLevel   obstacle_get_level(void);

#endif // OBSTACLE_DETECTOR_H
