#!/bin/bash

PRIVATE_KEY_PATH="$1"
PUBLIC_KEY_PATH="$2"

# private_key.pem 파일이 존재하는지 확인하고, 없으면 생성
if [ ! -f "$PRIVATE_KEY_PATH" ]; then
    echo "private_key.pem not found. Generating a new one."
    openssl genpkey -algorithm RSA -out "$PRIVATE_KEY_PATH" -pkeyopt rsa_keygen_bits:2048
else
    echo "private_key.pem found."
fi

# public.pem 파일이 존재하는지 확인하고, 없으면 생성
if [ ! -f "$PUBLIC_KEY_PATH" ]; then
    echo "public.pem not found. Generating a new one."
    openssl rsa -pubout -in "$PRIVATE_KEY_PATH" -out "$PUBLIC_KEY_PATH"
else
    echo "public.pem found."
fi