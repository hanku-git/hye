#include "signature.h"
#include <openssl/evp.h>
#include <openssl/pem.h>
#include <openssl/err.h>
#include <fstream>
#include <iostream>
#include <vector>

bool verifySignature(const std::string& dataPath, const std::string& signaturePath, const std::string& publicKeyPath) {
    // 파일을 읽기 위한 스트림
    std::ifstream dataFile(dataPath, std::ios::binary);
    std::ifstream signatureFile(signaturePath, std::ios::binary);
    FILE* publicKeyFile = fopen(publicKeyPath.c_str(), "r");

    if (!dataFile.is_open()) {
        //std::cerr << "Error opening data file: " << dataPath << std::endl;
        return false;
    }
    if (!signatureFile.is_open()) {
        //std::cerr << "Error opening signature file: " << signaturePath << std::endl;
        return false;
    }
    if (!publicKeyFile) {
        //std::cerr << "Error opening public key file: " << publicKeyPath << std::endl;
        return false;
    }

    // 파일 내용을 벡터에 저장
    std::vector<char> data((std::istreambuf_iterator<char>(dataFile)), std::istreambuf_iterator<char>());
    std::vector<char> signature((std::istreambuf_iterator<char>(signatureFile)), std::istreambuf_iterator<char>());

    // 공개 키 읽기
    EVP_PKEY* publicKey = PEM_read_PUBKEY(publicKeyFile, nullptr, nullptr, nullptr);
    fclose(publicKeyFile);

    if (!publicKey) {
        //std::cerr << "Error reading public key." << std::endl;
        return false;
    }

    // 서명 검증
    EVP_MD_CTX* ctx = EVP_MD_CTX_new();
    EVP_PKEY_CTX* pkeyCtx = nullptr;

    if (!EVP_DigestVerifyInit(ctx, &pkeyCtx, EVP_sha256(), nullptr, publicKey) ||
        !EVP_DigestVerifyUpdate(ctx, data.data(), data.size()) ||
        !EVP_DigestVerifyFinal(ctx, (unsigned char*)signature.data(), signature.size())) {
        //std::cerr << "Signature verification failed: " << ERR_error_string(ERR_get_error(), nullptr) << std::endl;
        EVP_MD_CTX_free(ctx);
        EVP_PKEY_free(publicKey);
        return false;
    }

    EVP_MD_CTX_free(ctx);
    EVP_PKEY_free(publicKey);

    return true;
}