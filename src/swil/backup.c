#include "backup/backup.h"

#include <stdio.h>

#define BKPSRAM_SIZE 4096  // bytes

_Static_assert(sizeof(Backup) < BKPSRAM_SIZE, "Backup does not fit in BKPSRAM");

static Backup s_backup;

Status backup_init() { printf("Initialized backup\n"); }

Backup* backup_get_ptr() { return &s_backup; }
