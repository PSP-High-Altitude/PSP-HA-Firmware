#include "backup/backup.h"

#include <stdio.h>

static Backup s_backup;

Status backup_init() {
    printf("Initialized backup\n");
    return STATUS_OK;
}

Backup* backup_get_ptr() { return &s_backup; }

Status backup_invalidate() {
    printf("Invalidated backup SRAM by clearing\n");
    memset(backup_get_ptr(), 0x00, sizeof(Backup));
    return STATUS_OK;
}
