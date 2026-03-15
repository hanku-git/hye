#ifndef SIGNATURE_HPP
#define SIGNATURE_HPP

#include <string>

bool verifySignature(const std::string& dataPath, const std::string& signaturePath, const std::string& publicKeyPath);

#endif // SIGNATURE_HPP