// #pragma once

// void InitCommunication();
// void ProcessCommunication();

// enum class StringType {
//   kInfo = 0,
//   kWarning = 1,
//   kError = 2
// };

// using ST = StringType;

// class SerialCommunication {
//   SerialCommunication();
//   void SendReply();
//   void SendData();
//   void SendString(StringType type, const char* msg);

//   void Update();
// private:
//   enum class SyncState {
//     kNotSynced = 0,
//     kSyncing = 1,
//     kSynced = 2
//   };

//   SyncState synced = false;
// };