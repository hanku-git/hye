#ifndef SQLITE_HELPER_HPP
#define SQLITE_HELPER_HPP

#include <sqlite3.h>
#include <openssl/evp.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <memory>
#include <set>
#include <algorithm>

struct UserInfo {
    int id;
    std::string username;
    std::string password;
};

// sqlite3 데이터베이스 연결을 위한 shared_ptr 타입 정의
using SqliteDatabasePtr = std::shared_ptr<sqlite3>;
namespace SqliteHelper
{
        // sqlite3_close 호출을 위한 커스텀 딜리터 정의
    struct SqliteDeleter {
        void operator()(sqlite3* db) const {
            sqlite3_close(db);
        }
    };


    // 데이터베이스 연결을 초기화하는 함수
    SqliteDatabasePtr connectDatabase(const std::string& databasePath);
    // Function to perform SHA-512 hashing
    std::string SHA512(const std::string& data); 
    // Function to create SQLite database table for users
    int createUserTable(SqliteDatabasePtr db);
    // Function to insert a new user into the database
    int insertUser(SqliteDatabasePtr db, const int id ,const std::string& username, const std::string& password);

    int deleteUser(SqliteDatabasePtr db, const int id);
    // Function to authenticate a user
    bool authenticateUser(SqliteDatabasePtr db, const std::string& username, const std::string& password);

    std::vector<UserInfo> selectUser(SqliteDatabasePtr db);

    int updateUser(SqliteDatabasePtr db, UserInfo info);

    std::vector<int> computeDifference(const std::vector<int>& setA, const std::vector<int>& setB);

    static int callback(void *data, int argc, char **argv, char **azColName) {
        std::vector<UserInfo>* users = static_cast<std::vector<UserInfo>*>(data);
        UserInfo user;
        user.id = std::stoi(argv[0]);
        user.username = argv[1] ? argv[1] : "";
        user.password = argv[2] ? argv[2] : "";
        users->push_back(user);
        return 0;
    }
}

#endif // SQLITE_HELPER_HPP