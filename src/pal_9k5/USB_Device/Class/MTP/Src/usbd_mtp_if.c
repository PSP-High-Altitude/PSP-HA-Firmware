/**
 ******************************************************************************
 * @file    usbd_mtp_if.c
 * @author  MCD Application Team
 * @brief   Source file for USBD MTP file list_files.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "usbd_mtp_if.h"

#include "FreeRTOS.h"
#include "dhara/map.h"
#include "fatfs/ff.h"
#include "nand_flash.h"
#include "task.h"
#include "timer.h"
#include "usbd_mtp_opt.h"

#define MTP_DRIVE_POINT NAND_MOUNT_POINT "/"

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

#define MTP_MAX_OBJECT_NUM 50

typedef struct {
    uint32_t Storage_id;
    uint16_t ObjectFormat;
    uint32_t ParentObject;
    uint32_t AssociationType;
    uint32_t AssociationDesc;
    uint32_t ObjectCompressedSize;
    uint8_t Filename[255];
} mtp_object_info;

typedef struct mtp_fnode {
    uint32_t handle;
    mtp_object_info *data;
    struct mtp_fnode *next;
    struct mtp_fnode *prev;
} mtp_file_node_t;

typedef struct {
    mtp_file_node_t *head;
    mtp_file_node_t *tail;
    uint32_t num_files;
} mtp_file_list_t;

mtp_file_list_t mtp_file_list = {.head = NULL, .tail = NULL, .num_files = 0};
uint32_t mtp_file_new_idx = 1;
uint32_t mtp_fs_size = 0;

uint32_t parent = 0;
uint32_t sc_buff[MTP_IF_SCRATCH_BUFF_SZE / 4U];
uint32_t sc_len = 0U;
uint32_t pckt_cnt = 1U;
uint32_t foldsize;

struct {
    uint8_t data[MTP_FILE_FIFO_SIZE];
    uint32_t head;
    uint32_t tail;
} mtp_file_fifo;
uint8_t mtp_current_operation = 0;  // 0: no operation, 1: read, 2: write

extern struct dhara_map s_map;
extern struct dhara_nand s_nand;
FIL curr_file;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static int add_file_node(mtp_file_list_t *list, uint32_t handle,
                         mtp_object_info *data) {
    if (list->num_files >= MTP_MAX_OBJECT_NUM) {
        free(data);
        return 1;
    }
    mtp_file_node_t *node = malloc(sizeof(mtp_file_node_t));
    if (node == NULL) {
        free(data);
        return 1;
    }
    node->data = data;
    node->next = NULL;
    node->prev = list->tail;
    if (list->tail != NULL) {
        list->tail->next = node;
    }
    list->tail = node;
    if (list->head == NULL) {
        list->head = node;
    }
    list->num_files++;
    return 0;
}

static void remove_file_node(mtp_file_list_t *list, uint32_t id) {
    mtp_file_node_t *node = list->tail;
    while (node != NULL) {
        if (node->handle == id) {
            if (node->prev != NULL) {
                node->prev->next = node->next;
            }
            if (node->next != NULL) {
                node->next->prev = node->prev;
            }
            if (list->head == node) {
                list->head = node->next;
            }
            if (list->tail == node) {
                list->tail = node->prev;
            }
            free(node->data);
            free(node);
            list->num_files--;
            return;
        }
        node = node->prev;
    }
}

static mtp_file_node_t *get_file_node(mtp_file_list_t *list, uint32_t id) {
    mtp_file_node_t *node = list->tail;
    while (node != NULL) {
        if (node->handle == id) {
            return node;
        }
        node = node->prev;
    }
    return NULL;
}

static void delete_file_list(mtp_file_list_t *list) {
    mtp_file_node_t *node = list->tail;
    while (node != NULL) {
        mtp_file_node_t *next = node->prev;
        free(node->data);
        free(node);
        node = next;
    }
    list->num_files = 0;
}

static uint8_t USBD_MTP_Itf_Init(void);
static uint8_t USBD_MTP_Itf_DeInit(void);
static uint32_t USBD_MTP_Itf_ReadData(uint32_t Param1, uint8_t *buff,
                                      MTP_DataLengthTypeDef *data_length,
                                      USBD_HandleTypeDef *pdev);
static uint16_t USBD_MTP_Itf_Create_NewObject(MTP_ObjectInfoTypeDef ObjectInfo,
                                              uint32_t objhandle);

static uint32_t USBD_MTP_Itf_GetIdx(uint32_t Param3, uint32_t *obj_handle);
static uint32_t USBD_MTP_Itf_GetParentObject(uint32_t Param);
static MTP_ObjectInfoTypeDef USBD_MTP_Itf_GetObjectInfo(uint32_t Param);
static uint8_t USBD_MTP_Itf_GetObjectName_len(uint32_t Param);
static void USBD_MTP_Itf_GetObjectName(uint32_t Param, uint8_t obj_len,
                                       uint16_t *buf);
static uint32_t USBD_MTP_Itf_GetObjectSize(uint32_t Param);
static uint64_t USBD_MTP_Itf_GetMaxCapability(void);
static uint64_t USBD_MTP_Itf_GetFreeSpaceInBytes(void);
static uint32_t USBD_MTP_Itf_GetNewIndex(uint16_t objformat);
static void USBD_MTP_Itf_WriteData(uint16_t len, uint8_t *buff);
static uint32_t USBD_MTP_Itf_GetContainerLength(uint32_t Param1);
static uint16_t USBD_MTP_Itf_DeleteObject(uint32_t Param1);

static void USBD_MTP_Itf_Cancel(uint32_t Phase);
/* static uint32_t USBD_MTP_Get_idx_to_delete(uint32_t Param, uint8_t *tab); */

USBD_MTP_ItfTypeDef USBD_MTP_fops = {
    USBD_MTP_Itf_Init,
    USBD_MTP_Itf_DeInit,
    USBD_MTP_Itf_ReadData,
    USBD_MTP_Itf_Create_NewObject,
    USBD_MTP_Itf_GetIdx,
    USBD_MTP_Itf_GetParentObject,
    USBD_MTP_Itf_GetObjectInfo,
    USBD_MTP_Itf_GetObjectName_len,
    USBD_MTP_Itf_GetObjectName,
    USBD_MTP_Itf_GetObjectSize,
    USBD_MTP_Itf_GetMaxCapability,
    USBD_MTP_Itf_GetFreeSpaceInBytes,
    USBD_MTP_Itf_GetNewIndex,
    USBD_MTP_Itf_WriteData,
    USBD_MTP_Itf_GetContainerLength,
    USBD_MTP_Itf_DeleteObject,
    USBD_MTP_Itf_Cancel,
    sc_buff,
    MTP_IF_SCRATCH_BUFF_SZE,
};

/* Private functions ---------------------------------------------------------*/

static void push_to_fifo(uint8_t *data, uint32_t len) {
    for (int i = 0; i < len; i++) {
        if ((mtp_file_fifo.head + 1) % MTP_FILE_FIFO_SIZE ==
            mtp_file_fifo.tail) {
            // FIFO is full
            return;
        }
        mtp_file_fifo.data[mtp_file_fifo.head] = data[i];
        mtp_file_fifo.head = (mtp_file_fifo.head + 1) % MTP_FILE_FIFO_SIZE;
    }
}

static uint32_t pop_from_fifo(uint8_t *data, uint32_t len) {
    uint32_t bytes_read = 0;
    for (int i = 0; i < len; i++) {
        if (mtp_file_fifo.head == mtp_file_fifo.tail) {
            // FIFO is empty
            return bytes_read;
        }
        data[i] = mtp_file_fifo.data[mtp_file_fifo.tail];
        mtp_file_fifo.tail = (mtp_file_fifo.tail + 1) % MTP_FILE_FIFO_SIZE;
        bytes_read++;
    }
    return bytes_read;
}

static uint32_t get_fifo_size() {
    if (mtp_file_fifo.head >= mtp_file_fifo.tail) {
        return mtp_file_fifo.head - mtp_file_fifo.tail;
    } else {
        return MTP_FILE_FIFO_SIZE - mtp_file_fifo.tail + mtp_file_fifo.head;
    }
}

static void traverse_fs(char path[256], uint32_t parent) {
    DIR dir;
    if (f_opendir(&dir, path) != FR_OK) {
        return;
    }

    FILINFO info;
    while (1) {
        if (f_readdir(&dir, &info) != FR_OK) {
            f_closedir(&dir);
            return;
        }
        if (info.fname[0] == '\0') {
            f_closedir(&dir);
            return;
        }
        if (strcmp(info.fname, ".") == 0 || strcmp(info.fname, "..") == 0) {
            continue;
        }

        if ((info.fattrib & AM_DIR) == 0) {
            mtp_object_info *obj_info =
                (mtp_object_info *)malloc(sizeof(mtp_object_info));
            memcpy(obj_info->Filename, info.fname, strlen(info.fname) + 1);
            obj_info->Storage_id = MTP_STORAGE_ID;
            obj_info->ObjectFormat = MTP_OBJ_FORMAT_UNDEFINED;
            obj_info->ParentObject = parent == 0xFFFFFFFF ? 0xFFFFFFFF : parent;
            obj_info->AssociationType = 0;
            obj_info->AssociationDesc = 0;
            obj_info->ObjectCompressedSize = info.fsize;
            mtp_fs_size += info.fsize;
            if (parent != 0xFFFFFFFF) {
                mtp_object_info *parent_obj =
                    get_file_node(&mtp_file_list, parent)->data;
                if (parent_obj == NULL) {
                    return;
                }
                parent_obj->ObjectCompressedSize += info.fsize;
            }
            if (add_file_node(&mtp_file_list, mtp_file_new_idx++, obj_info) !=
                0) {
                f_closedir(&dir);
                return;
            }
        } else {
            mtp_object_info *obj_info =
                (mtp_object_info *)malloc(sizeof(mtp_object_info));
            memcpy(obj_info->Filename, info.fname, strlen(info.fname) + 1);
            obj_info->Storage_id = MTP_STORAGE_ID;
            obj_info->ObjectFormat = MTP_OBJ_FORMAT_ASSOCIATION;
            obj_info->ParentObject = parent == 0xFFFFFFFF ? 0xFFFFFFFF : parent;
            obj_info->AssociationType = 1;
            obj_info->AssociationDesc = 0;
            obj_info->ObjectCompressedSize = 0;
            if (add_file_node(&mtp_file_list, mtp_file_new_idx++, obj_info) !=
                0) {
                f_closedir(&dir);
                return;
            }

            traverse_fs(info.fname, obj_info->Storage_id);
        }
    }
    f_closedir(&dir);
}

/**
 * @brief  USBD_MTP_Itf_Init
 *         Initialize the file system Layer
 * @param  None
 * @retval status value
 */
static uint8_t USBD_MTP_Itf_Init(void) {
    traverse_fs(NAND_MOUNT_POINT, 0xFFFFFFFF);

    mtp_file_fifo.head = 0;
    mtp_file_fifo.tail = 0;

    return 0;
}

/**
 * @brief  USBD_MTP_Itf_DeInit
 *         Uninitialize the file system Layer
 * @param  None
 * @retval status value
 */
static uint8_t USBD_MTP_Itf_DeInit(void) {
    delete_file_list(&mtp_file_list);
    mtp_fs_size = 0;
    mtp_file_new_idx = 1;
    mtp_file_fifo.head = 0;
    mtp_file_fifo.tail = 0;

    return 0;
}

/**
 * @brief  USBD_MTP_Itf_GetIdx
 *         Get all object handle
 * @param  Param3: current object handle
 * @param  obj_handle: all objects handle (subfolders/files) in current object
 * @retval number of object handle in current object
 */
static uint32_t USBD_MTP_Itf_GetIdx(uint32_t Param3, uint32_t *obj_handle) {
    uint32_t count = 0U;
    mtp_file_node_t *node = mtp_file_list.head;
    while (node != NULL) {
        if (node->data->ParentObject == Param3) {
            obj_handle[count] = node->handle;
            count++;
        }
        node = node->next;
    }

    return count;
}

/**
 * @brief  USBD_MTP_Itf_GetParentObject
 *         Get parent object
 * @param  Param: object handle (object index)
 * @retval parent object
 */
static uint32_t USBD_MTP_Itf_GetParentObject(uint32_t Param) {
    mtp_object_info *obj = get_file_node(&mtp_file_list, Param)->data;
    if (obj == NULL) {
        return 0xFFFFFFFF;
    }

    return obj->ParentObject;
}
/**
 * @brief  USBD_MTP_Itf_GetObjectFormat
 *         Get object format
 * @param  Param: object handle (object index)
 * @retval object format
 */
static MTP_ObjectInfoTypeDef USBD_MTP_Itf_GetObjectInfo(uint32_t Param) {
    mtp_object_info *obj = get_file_node(&mtp_file_list, Param)->data;
    if (obj == NULL) {
        MTP_ObjectInfoTypeDef ret = {0};
        return ret;
    }

    MTP_ObjectInfoTypeDef object_info = {
        .Storage_id = obj->Storage_id,
        .ObjectFormat = obj->ObjectFormat,
        .ObjectCompressedSize = obj->ObjectCompressedSize,
        .AssociationType = obj->AssociationType,
        .AssociationDesc = obj->AssociationDesc,
        .Filename_len = strlen((char *)obj->Filename),
    };
    for (int i = 0; i < object_info.Filename_len; i++) {
        object_info.Filename[i] = obj->Filename[i];
    }

    return object_info;
}

/**
 * @brief  USBD_MTP_Itf_GetObjectName_len
 *         Get object name length
 * @param  Param: object handle (object index)
 * @retval object name length
 */
static uint8_t USBD_MTP_Itf_GetObjectName_len(uint32_t Param) {
    mtp_object_info *obj = get_file_node(&mtp_file_list, Param)->data;
    if (obj == NULL) {
        return 0;
    }

    return strlen((char *)obj->Filename) + 1;
}

/**
 * @brief  USBD_MTP_Itf_GetObjectName
 *         Get object name
 * @param  Param: object handle (object index)
 * @param  obj_len: length of object name
 * @param  buf: pointer to object name
 * @retval object size in SD card
 */
static void USBD_MTP_Itf_GetObjectName(uint32_t Param, uint8_t obj_len,
                                       uint16_t *buf) {
    mtp_object_info *obj = get_file_node(&mtp_file_list, Param)->data;
    if (obj == NULL) {
        return;
    }

    for (int i = 0; i < obj_len; i++) {
        buf[i] = obj->Filename[i];
    }
}

/**
 * @brief  USBD_MTP_Itf_GetObjectSize
 *         Get size of current object
 * @param  Param: object handle (object index)
 * @retval object size in SD card
 */
static uint32_t USBD_MTP_Itf_GetObjectSize(uint32_t Param) {
    mtp_object_info *obj = get_file_node(&mtp_file_list, Param)->data;
    if (obj == NULL) {
        return 0;
    }

    return obj->ObjectCompressedSize;
}

/**
 * @brief  USBD_MTP_Itf_Create_NewObject
 *         Create new object in SD card and store necessary information for
 * future use
 * @param  ObjectInfo: object information to use
 * @param  objhandle: object handle (object index)
 * @retval None
 */
static uint16_t USBD_MTP_Itf_Create_NewObject(MTP_ObjectInfoTypeDef ObjectInfo,
                                              uint32_t objhandle) {
    return 0x2001;
}

/**
 * @brief  USBD_MTP_Itf_GetMaxCapability
 *         Get max capability in SD card
 * @param  None
 * @retval max capability
 */
static uint64_t USBD_MTP_Itf_GetMaxCapability(void) {
    uint64_t max_cap =
        dhara_map_capacity(&s_map) * (1U << s_nand.log2_page_size);

    return max_cap;
}

/**
 * @brief  USBD_MTP_Itf_GetFreeSpaceInBytes
 *         Get free space in bytes in SD card
 * @param  None
 * @retval free space in bytes
 */
static uint64_t USBD_MTP_Itf_GetFreeSpaceInBytes(void) {
    uint64_t fs_free =
        dhara_map_capacity(&s_map) * (1U << s_nand.log2_page_size) -
        mtp_fs_size;

    return fs_free;
}

/**
 * @brief  USBD_MTP_Itf_GetNewIndex
 *         Create new object handle
 * @param  objformat: object format
 * @retval object handle
 */
static uint32_t USBD_MTP_Itf_GetNewIndex(uint16_t objformat) {
    // New object handle
    return mtp_file_new_idx++;
}

/**
 * @brief  USBD_MTP_Itf_WriteData
 *         Write file data to SD card
 * @param  len: size of data to write
 * @param  buff: data to write in SD card
 * @retval None
 */
static void USBD_MTP_Itf_WriteData(uint16_t len, uint8_t *buff) {}

/**
 * @brief  USBD_MTP_Itf_GetContainerLength
 *         Get length of generic container
 * @param  Param1: object handle
 * @retval length of generic container
 */
static uint32_t USBD_MTP_Itf_GetContainerLength(uint32_t Param1) {
    mtp_object_info *obj = get_file_node(&mtp_file_list, Param1)->data;
    if (obj == NULL) {
        return 0;
    }

    return obj->ObjectCompressedSize + MTP_CONT_HEADER_SIZE;
}

/**
 * @brief  USBD_MTP_Itf_DeleteObject
 *         delete object from SD card
 * @param  Param1: object handle (file/folder index)
 * @retval response code
 */
static uint16_t USBD_MTP_Itf_DeleteObject(uint32_t Param1) {
    mtp_object_info *obj = get_file_node(&mtp_file_list, Param1)->data;
    if (obj == NULL) {
        return 0x2002;
    }

    char path[256];
    snprintf(path, 256, "%s/%s", NAND_MOUNT_POINT, (char *)obj->Filename);

    if (obj->ObjectFormat == MTP_OBJ_FORMAT_ASSOCIATION) {
        mtp_file_node_t *node = mtp_file_list.head;
        while (node != NULL) {
            if (node->data->ParentObject == Param1) {
                if (USBD_MTP_Itf_DeleteObject(node->data->Storage_id) !=
                    0x2001) {
                    return 0x2002;
                }
            }
            node = node->next;
        }
        int ret = f_unlink(path);
        remove_file_node(&mtp_file_list, Param1);
        if (ret < 0) {
            return 0x2002;
        }
    } else {
        int ret = f_unlink(path);
        remove_file_node(&mtp_file_list, Param1);
        if (ret < 0) {
            return 0x2002;
        }
    }

    return 0x2001;
}

/**
 * @brief  USBD_MTP_Get_idx_to_delete
 *         Get all files/foldres index to delete with descending order ( max
 * depth)
 * @param  Param: object handle (file/folder index)
 * @param  tab: pointer to list of files/folders to delete
 * @retval Number of files/folders to delete
 */
/* static uint32_t USBD_MTP_Get_idx_to_delete(uint32_t Param, uint8_t *tab)
{
  uint32_t cnt = 0U;

  return cnt;
}
*/

/**
 * @brief  USBD_MTP_Itf_ReadData
 *         Read data from SD card
 * @param  Param1: object handle
 * @param  buff: pointer to data to be read
 * @param  temp_length: current data size read
 * @retval necessary information for next read/finish reading
 */
static uint32_t USBD_MTP_Itf_ReadData(uint32_t Param1, uint8_t *buff,
                                      MTP_DataLengthTypeDef *data_length,
                                      USBD_HandleTypeDef *pdev) {
    if (data_length->temp_length == 0) {
        // Get file info
        mtp_object_info *obj = get_file_node(&mtp_file_list, Param1)->data;
        if (obj == NULL) {
            return 0;
        }

        char path[256];
        snprintf(path, 256, "%s/%s", NAND_MOUNT_POINT, (char *)obj->Filename);
        // Open file on first read
        if (f_open(&curr_file, path, FA_READ) != FR_OK) {
            data_length->totallen = 0;
            return 0;
        }
        mtp_current_operation = 1;
    }

    USBD_MTP_HandleTypeDef *hmtp =
        (USBD_MTP_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

    // Read data
    uint32_t read_len;
    if (data_length->temp_length == 0) {
        read_len = (uint32_t)hmtp->MaxPcktLen - MTP_CONT_HEADER_SIZE;
    } else {
        read_len = (uint32_t)hmtp->MaxPcktLen;
    }

    uint32_t fifo_size = get_fifo_size();
    if (fifo_size < read_len) {
        // Force a read
        mtp_readwrite_file();
        fifo_size = get_fifo_size();
    }

    // Failure in reading file or reached the end too early
    if (fifo_size == 0) {
        data_length->readbytes = 0;
        return 1;
    }

    // Copy from the FIFO
    read_len = pop_from_fifo(buff, read_len);

    // Update data length
    data_length->readbytes = read_len;
    data_length->temp_length += read_len;

    // Check if all data read
    if (data_length->temp_length == data_length->totallen) {
        mtp_current_operation = 0;
        f_close(&curr_file);
    }

    return 0;
}

/**
 * @brief  USBD_MTP_Itf_Cancel
 *         Close opened folder/file while cancelling transaction
 * @param  MTP_ResponsePhase: MTP current state
 * @retval None
 */
static void USBD_MTP_Itf_Cancel(uint32_t Phase) {
    /* Make sure to close open file while canceling transaction */
    if (Phase == 0) {
        mtp_current_operation = 0;
        mtp_file_fifo.head = 0;
        mtp_file_fifo.tail = 0;
        f_close(&curr_file);
    }

    return;
}

void mtp_readwrite_file_task() {
    while (1) {
        // Make sure MTP read cannot interrupt the read/write
        __NVIC_DisableIRQ(OTG_HS_IRQn);
        mtp_readwrite_file();
        __NVIC_EnableIRQ(OTG_HS_IRQn);
        DELAY(20);
    }
}

void mtp_readwrite_file() {
    uint32_t fifo_size = get_fifo_size();

    if (mtp_current_operation == 1 && fifo_size < MTP_FILE_FIFO_SIZE - 1) {
        uint8_t
            temp[MIN(MTP_FILE_READ_SIZE, MTP_FILE_FIFO_SIZE - fifo_size - 1)];
        UINT read;
        FRESULT stat = f_read(
            &curr_file, temp,
            MIN(MTP_FILE_READ_SIZE, MTP_FILE_FIFO_SIZE - fifo_size - 1), &read);
        if (read < 0 || stat != FR_OK) {
            mtp_current_operation = 0;
            f_close(&curr_file);
            mtp_file_fifo.head = 0;
            mtp_file_fifo.tail = 0;
            return;
        }
        push_to_fifo(temp, read);
    }
}