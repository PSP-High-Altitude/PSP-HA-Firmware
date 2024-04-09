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

#include "nand_flash.h"
#include "timer.h"
#include "usbd_mtp_opt.h"

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

mtp_object_info mtp_files[MTP_MAX_OBJECT_NUM];
uint32_t mtp_file_idx = 0;
uint32_t mtp_fs_size = 0;

uint32_t parent = 0;
uint32_t sc_buff[MTP_IF_SCRATCH_BUFF_SZE / 4U];
uint32_t sc_len = 0U;
uint32_t pckt_cnt = 1U;
uint32_t foldsize;

int curr_file;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
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

/**
 * @brief  USBD_MTP_Itf_Init
 *         Initialize the file system Layer
 * @param  None
 * @retval status value
 */
static void traverse_fs(char path[LFS_NAME_MAX + 1], uint32_t parent) {
    yaffs_DIR *dir;
    dir = yaffs_opendir(path);
    if (dir == NULL) {
        return;
    }

    struct yaffs_dirent *info;
    while (1) {
        info = yaffs_readdir(dir);
        if (info == NULL) {
            yaffs_closedir(dir);
            return;
        }
        if (strcmp(info->d_name, ".") == 0 || strcmp(info->d_name, "..") == 0) {
            continue;
        }

        struct yaffs_stat stat;
        char str[128];
        if (snprintf(str, 128, "%s%s", path, info->d_name) < 0) {
            yaffs_closedir(dir);
            return;
        }
        if (yaffs_lstat(str, &stat) != 0) {
            yaffs_closedir(dir);
            return;
        }

        if ((stat.st_mode & S_IFMT) == S_IFREG) {
            if (mtp_file_idx >= MTP_MAX_OBJECT_NUM) {
                yaffs_closedir(dir);
                return;
            }
            memcpy(mtp_files[mtp_file_idx].Filename, info->d_name,
                   strlen(info->d_name) + 1);
            mtp_files[mtp_file_idx].Storage_id = mtp_file_idx + 1;
            mtp_files[mtp_file_idx].ObjectFormat = MTP_OBJ_FORMAT_UNDEFINED;
            mtp_files[mtp_file_idx].ParentObject =
                parent == 0xFFFFFFFF ? 0xFFFFFFFF : parent + 1;
            mtp_files[mtp_file_idx].AssociationType = 0;
            mtp_files[mtp_file_idx].AssociationDesc = 0;
            mtp_files[mtp_file_idx].ObjectCompressedSize = stat.st_size;
            mtp_fs_size += stat.st_size;
            if (parent != 0xFFFFFFFF) {
                mtp_files[parent].ObjectCompressedSize++;
            }
            mtp_file_idx++;
        } else {
            if (mtp_file_idx >= MTP_MAX_OBJECT_NUM) {
                yaffs_closedir(dir);
                return;
            }
            memcpy(mtp_files[mtp_file_idx].Filename, info->d_name,
                   strlen(info->d_name) + 1);
            mtp_files[mtp_file_idx].Storage_id = mtp_file_idx + 1;
            mtp_files[mtp_file_idx].ObjectFormat = MTP_OBJ_FORMAT_ASSOCIATION;
            mtp_files[mtp_file_idx].ParentObject =
                parent == 0xFFFFFFFF ? 0xFFFFFFFF : parent + 1;
            mtp_files[mtp_file_idx].AssociationType = 1;
            mtp_files[mtp_file_idx].AssociationDesc = 0;
            mtp_files[mtp_file_idx].ObjectCompressedSize = 0;
            mtp_file_idx++;

            traverse_fs(info->d_name, mtp_file_idx - 1);
        }
    }
    yaffs_closedir(dir);
}

static uint8_t USBD_MTP_Itf_Init(void) {
    memset(mtp_files, 0, sizeof(mtp_files));
    mtp_files[0].Storage_id = mtp_file_idx + 1;
    mtp_files[0].ObjectFormat = MTP_OBJ_FORMAT_ASSOCIATION;
    mtp_files[0].ParentObject = 0;
    mtp_files[0].AssociationType = 1;
    mtp_files[0].AssociationDesc = 0;
    mtp_files[0].ObjectCompressedSize = 0;
    traverse_fs("/", 0xFFFFFFFF);
    return 0;
}

/**
 * @brief  USBD_MTP_Itf_DeInit
 *         Uninitialize the file system Layer
 * @param  None
 * @retval status value
 */
static uint8_t USBD_MTP_Itf_DeInit(void) { return 0; }

/**
 * @brief  USBD_MTP_Itf_GetIdx
 *         Get all object handle
 * @param  Param3: current object handle
 * @param  obj_handle: all objects handle (subfolders/files) in current object
 * @retval number of object handle in current object
 */
static uint32_t USBD_MTP_Itf_GetIdx(uint32_t Param3, uint32_t *obj_handle) {
    uint32_t count = 0U;
    for (int i = Param3; i < MTP_MAX_OBJECT_NUM; i++) {
        if (mtp_files[i].ParentObject == Param3) {
            obj_handle[count] = i + 1;
            count++;
        }
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
    uint32_t idx = Param - 1;

    return mtp_files[idx].ParentObject;
}
/**
 * @brief  USBD_MTP_Itf_GetObjectFormat
 *         Get object format
 * @param  Param: object handle (object index)
 * @retval object format
 */
static MTP_ObjectInfoTypeDef USBD_MTP_Itf_GetObjectInfo(uint32_t Param) {
    uint32_t idx = Param - 1;

    MTP_ObjectInfoTypeDef object_info = {
        .Storage_id = mtp_files[idx].Storage_id,
        .ObjectFormat = mtp_files[idx].ObjectFormat,
        .ObjectCompressedSize = mtp_files[idx].ObjectCompressedSize,
        .AssociationType = mtp_files[idx].AssociationType,
        .AssociationDesc = mtp_files[idx].AssociationDesc,
        .Filename_len = strlen((char *)mtp_files[idx].Filename),
    };
    for (int i = 0; i < object_info.Filename_len; i++) {
        object_info.Filename[i] = mtp_files[idx].Filename[i];
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
    uint32_t idx = Param - 1;

    return strlen((char *)mtp_files[idx].Filename) + 1;
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
    uint32_t idx = Param - 1;

    for (int i = 0; i < obj_len; i++) {
        buf[i] = mtp_files[idx].Filename[i];
    }
}

/**
 * @brief  USBD_MTP_Itf_GetObjectSize
 *         Get size of current object
 * @param  Param: object handle (object index)
 * @retval object size in SD card
 */
static uint32_t USBD_MTP_Itf_GetObjectSize(uint32_t Param) {
    uint32_t idx = Param - 1;

    return mtp_files[idx].ObjectCompressedSize;
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
    uint16_t rep_code = 0U;  // Read-only

    return rep_code;
}

/**
 * @brief  USBD_MTP_Itf_GetMaxCapability
 *         Get max capability in SD card
 * @param  None
 * @retval max capability
 */
static uint64_t USBD_MTP_Itf_GetMaxCapability(void) {
    uint64_t max_cap =
        MT29F4G_BLOCK_COUNT * MT29F4G_PAGE_PER_BLOCK * MT29F4G_PAGE_SIZE;

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
        (MT29F4G_BLOCK_COUNT * MT29F4G_PAGE_PER_BLOCK * MT29F4G_PAGE_SIZE) -
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
    return mtp_file_idx + 1;
}

/**
 * @brief  USBD_MTP_Itf_WriteData
 *         Write file data to SD card
 * @param  len: size of data to write
 * @param  buff: data to write in SD card
 * @retval None
 */
static void USBD_MTP_Itf_WriteData(uint16_t len, uint8_t *buff) {
    UNUSED(len);
    UNUSED(buff);

    return;
}

/**
 * @brief  USBD_MTP_Itf_GetContainerLength
 *         Get length of generic container
 * @param  Param1: object handle
 * @retval length of generic container
 */
static uint32_t USBD_MTP_Itf_GetContainerLength(uint32_t Param1) {
    uint32_t idx = Param1 - 1;

    return mtp_files[idx].ObjectCompressedSize + MTP_CONT_HEADER_SIZE;
}

/**
 * @brief  USBD_MTP_Itf_DeleteObject
 *         delete object from SD card
 * @param  Param1: object handle (file/folder index)
 * @retval response code
 */
static uint16_t USBD_MTP_Itf_DeleteObject(uint32_t Param1) {
    int idx = Param1 - 1;
    if (mtp_files[idx].ObjectFormat == MTP_OBJ_FORMAT_ASSOCIATION) {
        for (int i = idx; i < mtp_file_idx; i++) {
            if (mtp_files[i].ParentObject == Param1) {
                if (USBD_MTP_Itf_DeleteObject(i + 1) != 0x2001) {
                    return 0x2002;
                }
            }
        }
        int ret = yaffs_unlink((char *)mtp_files[idx].Filename);
        if (ret < 0) {
            return 0x2002;
        }
    } else {
        int ret = yaffs_unlink((char *)mtp_files[idx].Filename);
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
        // Open file on first read
        curr_file = yaffs_open((char *)mtp_files[Param1 - 1].Filename, O_RDONLY,
                               S_IREAD | S_IWRITE);
        if (curr_file < 0) {
            data_length->totallen = 0;
            return 0;
        }
    }

    // Go to last read position
    if (yaffs_lseek(curr_file, data_length->temp_length, SEEK_SET) < 0) {
        yaffs_close(curr_file);
        data_length->readbytes = 0;
        return 0;
    }

    USBD_MTP_HandleTypeDef *hmtp =
        (USBD_MTP_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

    // Read data
    int read;
    if (data_length->temp_length == 0) {
        read = yaffs_read(curr_file, buff,
                          (uint32_t)hmtp->MaxPcktLen - MTP_CONT_HEADER_SIZE);
    } else {
        read = yaffs_read(curr_file, buff, (uint32_t)hmtp->MaxPcktLen);
    }
    if (read < 0) {
        yaffs_close(curr_file);
        data_length->readbytes = 0;
        return 0;
    }

    // Increment read position
    data_length->temp_length += read;
    data_length->readbytes = read;

    // Check if all data read
    if (data_length->temp_length == data_length->totallen) {
        yaffs_close(curr_file);
        return 0;
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
        yaffs_close(curr_file);
    }

    return;
}
