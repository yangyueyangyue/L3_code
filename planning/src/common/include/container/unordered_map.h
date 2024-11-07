/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       unordered_map.h
 * @brief      Store data based on keywords
 * @details
 *
 * @author     longjiaoy
 * @date       2020.04.22
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/04/28  <td>1.0      <td>longjiaoy     <td>First edition
 * </table>
 *
 ******************************************************************************/
#ifndef PHOENIX_COMMON_UNORDEERED_MAP_H_
#define PHOENIX_COMMON_UNORDEERED_MAP_H_


#include <string>
#include "utils/log.h"
#include "container/static_vector.h"
#include "container/data_pool.h"
namespace phoenix {
namespace common {
/**
 * @struct HashKEY
 * @brief  Key value conversion template.
 */
template<typename KeyType>
struct HashKEY {
  /**
   * @brief define operator().
   * @param[in] key  the stored data's keyword.
   * @return key.
   */
  Uint32_t operator()(const KeyType& key) {
    return key;
  }
};

/**
 * @struct HashKEY<Int32_t>
 * @brief  Instantiate the template for type of Int32_t.
 */
template<>
struct HashKEY<Int32_t> {
  /**
   * @brief Define operator().
   * @param[in] key  the stored data's keyword.
   * @return key.
   */
  Uint32_t FIBOKEY(const Int32_t& n) {
    const Uint32_t fibonacci = 2654435769;
    Uint32_t hash = n;
    hash = (hash * fibonacci) >> 18;
    return hash;
  }
  Uint32_t operator()(const Int32_t& key) {
    return FIBOKEY(key);
  }
};

/**
 * @struct HashKEY<Uint64_t>
 * @brief  Instantiate the template for type of Uint64_t.
 */
template<>
struct HashKEY<Uint64_t> {
  /**
   * @brief Define operator().
   * @param[in] key  the stored data's keyword.
   * @return key.
   */
  Uint32_t FIBOKEY(const Uint64_t& n) {
    const Uint64_t fibonacci = 11400714819323198485ULL;
    Uint32_t hash = static_cast<Uint32_t>((n * fibonacci) >> 18);
    return hash;
  }
  Uint32_t operator()(const Uint64_t& key) {
    return FIBOKEY(key);
  }
};

/**
 * @struct HashKEY<char*>
 * @brief  Instantiate the template for type of string.
 */
template<>
struct HashKEY<Char_t*> {
  /**
   * @brief Change string into unsigned type of int.
   * @param[in] key  the stored data's keyword.
   * @return Conversion result
   */
  Uint32_t BKDRHash(const Char_t* str) {
    Uint32_t hash = 0;
    while (*str) {
      hash = hash * 31 + *str;
      ++str;
    }
    return hash;
  }
  Uint32_t operator()(const Char_t* key) {
    return BKDRHash(key);
  }
};

/**
 * @struct HashKEY<std::string>
 * @brief  Instantiate the template for type of string.
 */
template<>
struct HashKEY<std::string> {
  /**
   * @brief Change string into unsigned type of int.
   * @param[in] key  the stored data's keyword.
   * @return Conversion result
   */
  Uint32_t BKDRHash(const std::string& str) {
    Uint32_t hash = 0;
    for (std::string::const_iterator it = str.begin(); it != str.end(); ++it) {
      hash = hash * 31 + *it;
    }
    return hash;
  }
  Uint32_t operator()(const std::string& key) {
    return BKDRHash(key);
  }
};

/**
 * @class UnorderedMap
 * @brief An associative container, which uses a hash table \n
 *         structure internally, has a fast retrieval function.
 * @param KeyType The type of user's keyword.
 * @param DataType The type of user's data.
 * @param DataNum The max size of the storage used to store user's data.
 *
 */
template<typename KeyType, typename DataType, Int32_t DataNum>
class UnorderedMap {
public:
  /**
   * @brief Contructor.
   */
  UnorderedMap() {
    Clear();
  }

  /**
   * @brief clear.
   */
  void Clear() {
    storage_.Clear();
    buckets_.Resize(DataNum);
    for (Int32_t i = 0; i < DataNum; ++i) {
      buckets_[i].Init();
    }
  }

  /**
   * @brief Insert (key,data) in bucket list
   * @param[in] key user's keyword.
   * @param[in] data user's data.
   * @return true,insert success\n
   *         false,insert failed
   */
  bool Insert(const KeyType& key, const DataType& data);

  /**
   * @brief Allocate storage space for keywords
   * @param[in] key user's keyword.
   * @return  Data storage address
   */
  DataType* Allocate(const KeyType& key);

  /**
   *@brief Find the key corresponding data
   *@param[in] key user's keyword.
   *@return  Data user's data.
   */
  const DataType* Find(const KeyType& key) const;

  /**
   * @brief Erase storage based on keywords
   * @param[in] key user's keyword.
   * @return true,erase success\n
   *         false,erase failed
   */
  bool Erase(const KeyType& key);

  /**
   * @brief Get all data from this map
   * @param[out] data the data from map
   */
  void GetAll(StaticVector<DataType, DataNum>* data) const;

private:
  /*
   * @brief Calculate storage index in bucket list by key
   * @param[in] key user's keyword.
   * @return index of bucket list.
   */
  Uint32_t CalcIndexByKey(const KeyType& key) const {
    // Get the converted key
    HashKEY<KeyType> hK;
    Uint32_t hash = hK(key);
    // hash ^= (hash >> 20) ^ (hash >> 12);
    // hash = hash ^ (hash >> 7) ^ (hash >> 4);
    // // Get index in bucket list
    // Int32_t index = hash & (DataNum - 1);
    // return index;
    return (hash % DataNum);
  }

private:
  // Define the type of node of list associated with storage_
  struct StorageNode {
    // Next is the index of next node
    Int32_t next;
    // The user's keyword
    KeyType key;
    // The user's data
    DataType data;

    void Init() {
      next = -1;
    }

    StorageNode() {
      Init();
    }
  };

  // Define the type of node of list associated with bucket_
  struct BucketNode {
    // The index  referenced to  storage_
    Int32_t storage_offset;

    void Init() {
      storage_offset = -1;
    }

    BucketNode() {
      Init();
    }
  };

private:
  // Data poll stored user's (key,data)
  DataPool<StorageNode, DataNum> storage_;

  // Array stored StorageNode
  StaticVector<BucketNode, DataNum> buckets_;
};


template<typename KeyType, typename DataType, Int32_t DataNum>
bool UnorderedMap<KeyType, DataType, DataNum>::Insert(
    const KeyType& key, const DataType& data) {
  // Get the index of bucket list which stored the node
  Uint32_t bucket_index = CalcIndexByKey(key);

  Int32_t offset = buckets_[bucket_index].storage_offset;
  while (!storage_.IsNull(offset)) {
    StorageNode* storage_node = storage_.GetVirtualAddr(offset);
    if (Nullptr_t == storage_node) {
      LOG_ERR << "Failed to get address from data pool.";
      return false;
    }
    if (storage_node->key == key) {
      storage_node->data = data;
      return true;
    }
    offset = storage_node->next;
  }

  // Get offset ( data pool) which will be stored in bucket list
  offset = storage_.Allocate();
  if (storage_.IsNull(offset)) {
    LOG_ERR << "The storage is full.";
    return false;
  }
  StorageNode* storage_node = storage_.GetVirtualAddr(offset);
  if (Nullptr_t == storage_node) {
    LOG_ERR << "Failed to get address from data pool.";
    return false;
  }
  storage_node->next = -1;
  storage_node->key = key;
  storage_node->data = data;

  if (storage_.IsNull(buckets_[bucket_index].storage_offset)) {
    // No element in this bucket
    buckets_[bucket_index].storage_offset = offset;
  } else {
    // Insert the storage node to the head of bucket list
    storage_node->next = buckets_[bucket_index].storage_offset;
    buckets_[bucket_index].storage_offset = offset;
  }

  return true;
}

template<typename KeyType, typename DataType, Int32_t DataNum>
const DataType* UnorderedMap<KeyType, DataType, DataNum>::Find(
    const KeyType& key) const {
  // Get the index of bucket list which stored the node
  Uint32_t bucket_index = CalcIndexByKey(key);

  // Get the first offset of data pool  which stored in bucket
  Int32_t offset = buckets_[bucket_index].storage_offset;

  while (!storage_.IsNull(offset)) {
    // Get the storage_node
    const StorageNode* storage_node = storage_.GetVirtualAddr(offset);
    if (Nullptr_t == storage_node) {
      LOG_ERR << "Failed to get address from data pool.";
      return Nullptr_t;
    }
    if (storage_node->key == key) {
      // Find the data,return
      return &(storage_node->data);
    }
    offset = storage_node->next;
  }

  return Nullptr_t;
}


template<typename KeyType, typename DataType, Int32_t DataNum>
bool UnorderedMap<KeyType, DataType, DataNum>::Erase(
    const KeyType& key) {
  // Get the index in bucket_.
  Uint32_t bucket_index = CalcIndexByKey(key);

  // Get the first offset of data pool  which stored in bucket
  Int32_t offset = buckets_[bucket_index].storage_offset;

  StorageNode* storage_node = storage_.GetVirtualAddr(offset);
  StorageNode* storage_node_pre = storage_.GetVirtualAddr(offset);

  while (!storage_.IsNull(offset)) {
    // Get the storage_node
    storage_node = storage_.GetVirtualAddr(offset);

    if (Nullptr_t == storage_node) {
      LOG_ERR << "Failed to get address from data pool.";
      return false;
    }
    if (storage_node->key == key) {
      Int32_t next_offset = storage_node->next;
      if (offset == buckets_[bucket_index].storage_offset) {
        buckets_[bucket_index].storage_offset = next_offset;
      } else {
        storage_node_pre->next = next_offset;
      }
      storage_.Deallocate(offset);
      return true;
    } else {
      storage_node_pre = storage_node;
      offset = storage_node->next;
    }
  }
  return false;
}

template<typename KeyType, typename DataType, Int32_t DataNum>
DataType* UnorderedMap<KeyType, DataType, DataNum>::Allocate(
    const KeyType& key) {
  Uint32_t bucket_index = CalcIndexByKey(key);
  Int32_t offset = buckets_[bucket_index].storage_offset;

  while (!storage_.IsNull(offset)) {
    StorageNode* storage_node = storage_.GetVirtualAddr(offset);
    if (Nullptr_t == storage_node) {
      LOG_ERR << "Failed to get address from data pool.";
      return Nullptr_t;
    }
    if (storage_node->key == key) {
      return &(storage_node->data);
    }
    offset = storage_node->next;
  }

  offset = storage_.Allocate();
  if (storage_.IsNull(offset)) {
    // Data pool is full
    LOG_ERR << "The storage is full.";
    return Nullptr_t;
  }

  // Allocate the node in storage_
  StorageNode* storage_node = storage_.GetVirtualAddr(offset);
  if (Nullptr_t == storage_node) {
    LOG_ERR << "Failed to get address from data pool.";
    return Nullptr_t;
  }
  storage_node->next = -1;
  storage_node->key = key;

  /* DOO1 fuyuanyi 2023-04-27 (begin) */
  /* 修改编译warning问题（无符号与0比较） */
  /* DOO1 fuyuanyi 2023-04-27 (end) */

  if (storage_.IsNull(buckets_[bucket_index].storage_offset)) {
    // No element in this bucket
    buckets_[bucket_index].storage_offset = offset;
  } else {
    // Allocate the storage node to the head of bucket list
    storage_node->next = buckets_[bucket_index].storage_offset;
    buckets_[bucket_index].storage_offset = offset;
  }

  return &(storage_node->data);
}

template<typename KeyType, typename DataType, Int32_t DataNum>
void UnorderedMap<KeyType, DataType, DataNum>::GetAll(
    StaticVector<DataType, DataNum>* data) const {
  for (Int32_t i = 0; i < buckets_.Size(); ++i) {
    Int32_t offset = buckets_[i].storage_offset;
    if (offset >= 0) {
      while (!storage_.IsNull(offset)) {
        // Get the storage_node
        const StorageNode* storage_node = storage_.GetVirtualAddr(offset);
        if (Nullptr_t == storage_node) {
          LOG_ERR << "Failed to get address from data pool.";
          break;
        }
        data->PushBack(storage_node->data);

        offset = storage_node->next;
      }
    }
  }
}


}  // namespace common
}  // namespace phoenix

#endif  //  PHOENIX_COMMON_UNORDEERED_MAP_H_
