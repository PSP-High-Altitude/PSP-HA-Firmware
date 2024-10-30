#include "backup/backup.h"

#include <stdio.h>

static Backup s_backup;

Status backup_init() {
    printf("Initialized backup\n");
    return STATUS_OK;
}

Backup* backup_get_ptr() { return &s_backup; }
