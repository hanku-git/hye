#include "sqliteHelper.hpp"

SqliteDatabasePtr SqliteHelper::connectDatabase(const std::string& databasePath) {
    sqlite3* dbRaw = nullptr;
    int rc = sqlite3_open(databasePath.c_str(), &dbRaw);
    if (rc != SQLITE_OK) {
        std::cerr << "Cannot open database: " << sqlite3_errmsg(dbRaw) << std::endl;
        sqlite3_close(dbRaw);
        return nullptr;
    }
    return SqliteDatabasePtr(dbRaw, SqliteDeleter());
}

std::string SqliteHelper::SHA512(const std::string& data) {
    EVP_MD_CTX* ctx = EVP_MD_CTX_new();
    if (ctx == nullptr) {
        // Handle error
        return "";
    }

    if (EVP_DigestInit_ex(ctx, EVP_sha512(), nullptr) != 1) {
        // Handle error
        EVP_MD_CTX_free(ctx);
        return "";
    }

    if (EVP_DigestUpdate(ctx, data.c_str(), data.size()) != 1) {
        // Handle error
        EVP_MD_CTX_free(ctx);
        return "";
    }

    unsigned char hash[EVP_MAX_MD_SIZE];
    unsigned int hashLen;
    if (EVP_DigestFinal_ex(ctx, hash, &hashLen) != 1) {
        // Handle error
        EVP_MD_CTX_free(ctx);
        return "";
    }

    EVP_MD_CTX_free(ctx);

    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (unsigned int i = 0; i < hashLen; ++i) {
        ss << std::setw(2) << static_cast<unsigned>(hash[i]);
    }
    return ss.str();
}

int SqliteHelper::createUserTable(SqliteDatabasePtr db) {
    const char* createTableSQL = "CREATE TABLE IF NOT EXISTS users (id INTEGER PRIMARY KEY, username TEXT UNIQUE NOT NULL, password TEXT NOT NULL)";
    char* errorMessage;
    int result = sqlite3_exec(db.get(), createTableSQL, NULL, NULL, &errorMessage);
    if (result != SQLITE_OK) {
        std::cerr << "Error creating table: " << errorMessage << std::endl;
        sqlite3_free(errorMessage);
        return result;
    }
    return SQLITE_OK;
}

int SqliteHelper::insertUser(SqliteDatabasePtr db, const int id, const std::string& username, const std::string& password) {
    std::string hashedPassword = SHA512(password);
    if (hashedPassword.empty()) {
        std::cerr << "Error hashing password." << std::endl;
        return SQLITE_ERROR;
    }

    std::string insertSQL = "INSERT INTO users (id, username, password) VALUES (?, ?, ?)";
    sqlite3_stmt* statement;
    int result = sqlite3_prepare_v2(db.get(), insertSQL.c_str(), -1, &statement, NULL);
    if (result != SQLITE_OK) {
        std::cerr << "Error preparing statement: " << sqlite3_errmsg(db.get()) << std::endl;
        return result;
    }
    sqlite3_bind_int(statement, 1, id);
    sqlite3_bind_text(statement, 2, username.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(statement, 3, hashedPassword.c_str(), -1, SQLITE_STATIC);
    std::cout << "User hashed Password ::" << hashedPassword.c_str() << std::endl;

    result = sqlite3_step(statement);
    if (result != SQLITE_DONE) {
        std::cerr << "Error inserting user: " << sqlite3_errmsg(db.get()) << std::endl;
        sqlite3_finalize(statement);
        return result;
    }
    sqlite3_finalize(statement);
    std::cout << "User inserted successfully." << std::endl;
    return SQLITE_OK;
}

int SqliteHelper::deleteUser(SqliteDatabasePtr db, const int id) {
    
    std::string deleteSQL = "DELETE FROM users WHERE id = ?";
    sqlite3_stmt* statement;
    int result = sqlite3_prepare_v2(db.get(), deleteSQL.c_str(), -1, &statement, NULL);
    if (result != SQLITE_OK) {
        std::cerr << "Error preparing statement: " << sqlite3_errmsg(db.get()) << std::endl;
        return result;
    }
    sqlite3_bind_int(statement, 1, id);

    result = sqlite3_step(statement);
    if (result != SQLITE_DONE) {
        std::cerr << "Error delete user: " << sqlite3_errmsg(db.get()) << std::endl;
        sqlite3_finalize(statement);
        return result;
    }
    sqlite3_finalize(statement);
    std::cout << "User deleted successfully." << std::endl;
    return SQLITE_OK;
}

bool SqliteHelper::authenticateUser(SqliteDatabasePtr db, const std::string& username, const std::string& password) {
    std::string hashedPassword = SHA512(password);

    std::string selectSQL = "SELECT password FROM users WHERE username = ?";
    sqlite3_stmt* statement;
    int result = sqlite3_prepare_v2(db.get(), selectSQL.c_str(), -1, &statement, NULL);
    if (result != SQLITE_OK) {
        std::cerr << "Error preparing statement: " << sqlite3_errmsg(db.get()) << std::endl;
        return false;
    }
    sqlite3_bind_text(statement, 1, username.c_str(), -1, SQLITE_STATIC);

    result = sqlite3_step(statement);
    if (result == SQLITE_ROW) {
        std::string storedPassword = reinterpret_cast<const char*>(sqlite3_column_text(statement, 0));
        sqlite3_finalize(statement);
        return hashedPassword == storedPassword;
    } else {
        std::cerr << "Error retrieving user: " << sqlite3_errmsg(db.get()) << std::endl;
        sqlite3_finalize(statement);
        return false;
    }
}

std::vector<UserInfo> SqliteHelper::selectUser(SqliteDatabasePtr db) {
    char *zErrMsg = 0;
    int rc;
    const char *sql;
    std::vector<UserInfo> users;

    sql = "SELECT id, username, password FROM users order by id";
    rc = sqlite3_exec(db.get(), sql, callback, (void*)&users, &zErrMsg);
    if( rc != SQLITE_OK ) {
        std::cerr << "SQL error: " << zErrMsg << std::endl;
        sqlite3_free(zErrMsg);
    } else {
        std::cout << "Operation done successfully" << std::endl;
    }
    // 결과 출력
    for(const auto& user : users) {
        std::cout << "ID: " << user.id << " Username: " << user.username << " Password: " << user.password << std::endl;
    }
    
    return users;
    sqlite3_close(db.get());
}

int SqliteHelper::updateUser(SqliteDatabasePtr db, UserInfo info) {
    char* errMsg = 0;
    int rc;
    sqlite3_stmt* stmt;
    std::string update_sql = "UPDATE users SET username = ?, password = ? WHERE id = ?";
    sqlite3_prepare_v2(db.get(), update_sql.c_str(), -1, &stmt, 0);
    sqlite3_bind_text(stmt, 1, info.username.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, info.password.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_int(stmt, 3, info.id);
    rc = sqlite3_step(stmt);
    if (rc != SQLITE_DONE) {
        std::cerr << "update failed: " << sqlite3_errmsg(db.get()) << std::endl;
        return 1;
    }
    sqlite3_finalize(stmt);
    return 0;
}

std::vector<int> SqliteHelper::computeDifference(const std::vector<int>& vecA, const std::vector<int>& vecB) {
    std::vector<int> difference;
    std::set_difference(vecA.begin(), vecA.end(), vecB.begin(), vecB.end(),
                        std::back_inserter(difference));
    return difference;
}

