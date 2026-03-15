
# from typing import List
# from langchain_ollama.llms import OllamaLLM
# from langchain_core.prompts import FewShotChatMessagePromptTemplate, ChatPromptTemplate
# from langchain_core.example_selectors import SemanticSimilarityExampleSelector
# from langchain_ollama import OllamaEmbeddings  
# from langchain_chroma import Chroma
# import json

# # 소스 디스펜싱 프롬프트 설정
# SOURCE_PREFIX = """
#     사용자의 요청에 따라 소스 디스펜싱 명령어를 처리하세요.
#     이모티콘은 절대 사용하지 마세요.
#     설명은 생략하고 결과만 출력하세요.

#     1. 소스 종류:
#         - 케첩 (Ketchup)
#         - 치폴레 (Chipotle)
#         - 갈릭치즈 (Garlic Cheese)
#         - 과카몰리 (Guacamole)

#     2. 입력 및 출력 형식:
#         - 입력: 자연어로 표현된 요청 (예: "케첩소스를 담아줘", "치폴레 소스 넣어줘")
#         - 출력: { 'task': 'dispense', 'source': '소스명' } 형식으로 반환

#     3. 예시:
#         - 입력: "케첩소스를 담아줘"
#           출력: { 'task': 'dispense', 'source': 'Ketchup' }

#         - 입력: "과카몰리 소스를 넣어줘"
#           출력: { 'task': 'dispense', 'source': 'Guacamole' }
# """

# # 예제 데이터
# source_examples = [
#     {"input": "케첩소스를 담아줘", "output": "{'task': 'dispense', 'source': 'Ketchup'}"},
#     {"input": "치폴레 소스를 넣어줘", "output": "{'task': 'dispense', 'source': 'Chipotle'}"},
#     {"input": "갈릭치즈 소스를 담아줘", "output": "{'task': 'dispense', 'source': 'Garlic Cheese'}"},
#     {"input": "과카몰리 소스를 줘", "output": "{'task': 'dispense', 'source': 'Guacamole'}"}
# ]

# class LLMModel():

#     def __init__(self):
#         model = OllamaLLM(model="llama3.1", temperature=0.0)
#         embeddings = OllamaEmbeddings(model="llama3.1")

#         # 예제 프롬프트 설정
#         example_prompt = ChatPromptTemplate.from_messages([
#             ("human", "{input}"),
#             ("ai", "{output}")
#         ])

#         source_example_selector = SemanticSimilarityExampleSelector.from_examples(
#             source_examples, embeddings, Chroma, k=4
#         )

#         source_few_shot = FewShotChatMessagePromptTemplate(
#             example_selector=source_example_selector,
#             example_prompt=example_prompt,
#             input_variables=["input"]
#         )

#         source_prompt = ChatPromptTemplate.from_messages([
#             ("system", SOURCE_PREFIX),
#             source_few_shot,
#             ("human", "{input}")
#         ])

#         self.source_chain = source_prompt | model

#     def get_result(self, state, input):
#         try:
#             if state == "dispense":
#                 output = self.source_chain.invoke({"input": input})
#                 print("LLM 출력: ", output)  # ✅ 디버깅을 위해 추가
#                 result = json.loads(output.replace("'", '"'))  # ✅ 안정적인 JSON 파싱
#                 return result
#             else:
#                 return {'error': "알 수 없는 상태입니다."}

#         except Exception as e:
#             print(f"오류 발생: {str(e)}")
#             return {'error': "소스 디스펜싱 명령어를 인식하지 못했습니다."}

from typing import List
from langchain_ollama.llms import OllamaLLM
from langchain_core.prompts import FewShotChatMessagePromptTemplate, ChatPromptTemplate
from langchain_core.example_selectors import SemanticSimilarityExampleSelector
from langchain_ollama import OllamaEmbeddings  
from langchain_chroma import Chroma
import json

# ✅ 매우 상세하고 풍성한 시스템 프롬프트 (영어 및 한국어 버전 포함)
#    (추가로, 오타/발음 오류가 발생한 경우 fuzzy matching을 고려하도록 지시)
SOURCE_PREFIX = """
🤖 **You are an elite, state-of-the-art AI assistant for an automated robotic sauce dispensing system.**
Your mission is to meticulously analyze user input (which can be in either English or Korean) and determine which sauce to dispense, returning your result in a strictly formatted JSON structure.

**Essential Guidelines:**
1. **Strict Output Format:**  
   Your entire response MUST be a single JSON object with no additional commentary or formatting. Valid outputs are:
   - For a valid sauce request:
     ```json
     { "task": "dispense", "source": "<SauceName>" }
     ```
   - For an invalid, ambiguous, or unsupported request:
     ```json
     { "error": "Unable to recognize the requested sauce. Please try again." }
     ```
2. **Supported Sauces (소스 목록):**
   - **Ketchup (케첩)**
   - **Chipotle (치폴레)**
   - **Garlic Cheese (갈릭 치즈)**
   - **Guacamole (과카몰리)**
   *No other sauces are allowed. Any sauce request not exactly matching one of these must return an error.*
   
3. **Language Handling:**  
   User input may be provided in **English** or **Korean**. You must handle both languages with equal accuracy.

4. **User Request Specifications (사용자 요청 규칙):**
   - **Valid Examples (올바른 요청):**
     - "I would like some ketchup, please."
     - "케첩 소스 좀 주세요."
     - "Can you add garlic cheese sauce?"
     - "치폴레 소스를 선택할게요."
   - **Invalid/Unsupported Requests (부적절한 요청):**
     - "Please add some sauce." *(No specific sauce mentioned)*
     - "Give me the spicy sauce!" *(Sauce not in the list)*
     - "소스 추천해 주세요." *(Ambiguous request)*
     - "Do you have any extra sauces?" *(Unclear/unsupported)*
     
5. **Processing Rules:**
   - **Exact and Fuzzy Matching:**  
     Return exactly one of the allowed sauce names. If the sauce name in the user input is similar (based on fuzzy matching) to one of the supported sauces, choose the closest matching sauce.
   - **Error Handling:**  
     If the input is ambiguous, incomplete, or does not sufficiently match any allowed sauce, output the error JSON.
   - **No Extra Text:**  
     Do not output any text besides the JSON structure.

---
💡 **추가 한국어 안내:**
- 당신은 최첨단 자동화 로봇 소스 디스펜싱 시스템을 위한 AI 어시스턴트입니다.
- 사용자가 영어 또는 한국어로 요청한 소스 종류를 정확히 파악하여 아래와 같이 정해진 JSON 형식으로 응답해야 합니다.
  - 유효한 요청의 경우:
    ```json
    { "task": "dispense", "source": "<소스명>" }
    ```
  - 요청이 모호하거나 지원하지 않는 소스인 경우:
    ```json
    { "error": "Unable to recognize the requested sauce. Please try again." }
    ```
- 반드시 **케첩, 치폴레, 갈릭 치즈, 과카몰리** 중 하나만 반환해야 하며, 다른 소스는 생성하지 마세요.
- 추가 설명이나 부가 텍스트 없이 오직 JSON 형식만 출력해야 합니다.

Follow these instructions meticulously to ensure a precise, accurate, and error-free output.
"""

# ✅ 더욱 다양한 Few-Shot 학습 데이터 (영어/한국어 혼용 및 에지 케이스 포함)
source_examples = [
    {"input": "케첩 소스를 담아줘", "output": "{ \"task\": \"dispense\", \"source\": \"Ketchup\" }"},
    {"input": "치폴레 소스를 넣어줘", "output": "{ \"task\": \"dispense\", \"source\": \"Chipotle\" }"},
    {"input": "갈릭 치즈 소스를 주세요", "output": "{ \"task\": \"dispense\", \"source\": \"Garlic Cheese\" }"},
    {"input": "과카몰리 소스를 줘", "output": "{ \"task\": \"dispense\", \"source\": \"Guacamole\" }"},
    {"input": "I’d like some garlic cheese sauce, please.", "output": "{ \"task\": \"dispense\", \"source\": \"Garlic Cheese\" }"},
    {"input": "Can you add ketchup to my order?", "output": "{ \"task\": \"dispense\", \"source\": \"Ketchup\" }"},
    {"input": "Please add ketchup sauce.", "output": "{ \"task\": \"dispense\", \"source\": \"Ketchup\" }"},
    {"input": "소스를 추가하고 싶은데 어떤 게 있나요?", "output": "{ \"error\": \"Unable to recognize the requested sauce. Please try again.\" }"},
    {"input": "Give me spicy sauce!", "output": "{ \"error\": \"Unable to recognize the requested sauce. Please try again.\" }"},
    {"input": "Can I have some special sauce?", "output": "{ \"error\": \"Unable to recognize the requested sauce. Please try again.\" }"},
    {"input": "치즈소스 추가할래요!", "output": "{ \"task\": \"dispense\", \"source\": \"Garlic Cheese\" }"},
    {"input": "I want chipotle, thanks!", "output": "{ \"task\": \"dispense\", \"source\": \"Chipotle\" }"},
    {"input": "치폴레 주세요.", "output": "{ \"task\": \"dispense\", \"source\": \"Chipotle\" }"},
    {"input": "Please provide guacamole.", "output": "{ \"task\": \"dispense\", \"source\": \"Guacamole\" }"},
    {"input": "소스 좀 넣어줘.", "output": "{ \"error\": \"Unable to recognize the requested sauce. Please try again.\" }"},
    # 추가: STT 오인식 케이스 ("과감 올리" → Guacamole)
    {"input": "과감 올리", "output": "{ \"task\": \"dispense\", \"source\": \"Guacamole\" }"}
]

class LLMModel():
    def __init__(self):
        # LLM 및 임베딩 초기화 (temperature=0으로 결정론적 출력 보장)
        model = OllamaLLM(model="llama3.1", temperature=0.0)
        embeddings = OllamaEmbeddings(model="llama3.1")

        # Few-Shot 예제 프롬프트 템플릿 구성 (human, ai 메시지 형식)
        example_prompt = ChatPromptTemplate.from_messages([
            ("human", "{input}"),
            ("ai", "{output}")
        ])

        # Semantic Similarity 기반 예제 선택 (입력과 가장 유사한 예제 최대 5개 선택)
        source_example_selector = SemanticSimilarityExampleSelector.from_examples(
            source_examples, embeddings, Chroma, k=5
        )

        # Few-Shot Chat Message Prompt Template 생성 (선택된 예제를 기반으로 동적 프롬프트 구성)
        source_few_shot = FewShotChatMessagePromptTemplate(
            example_selector=source_example_selector,
            example_prompt=example_prompt,
            input_variables=["input"]
        )

        # 시스템 프롬프트, Few-Shot 예제, 사용자 입력을 포함하는 최종 프롬프트 체인 구성
        source_prompt = ChatPromptTemplate.from_messages([
            ("system", SOURCE_PREFIX),
            source_few_shot,
            ("human", "{input}")
        ])

        self.source_chain = source_prompt | model

    def get_result(self, state, input):
        """
        주어진 state에 따라 입력을 처리합니다.
        - state가 'dispense'인 경우, 프롬프트 체인을 호출하여 소스 디스펜싱 결과를 얻습니다.
        - 결과는 Python의 dict 형식으로 반환됩니다.
        """
        try:
            if state == "dispense":
                output = self.source_chain.invoke({"input": input})
                print("✅ LLM Output:", output)
                result = json.loads(output.replace("'", '"'))
                return result
            else:
                return {'error': "알 수 없는 상태입니다."}
        except Exception as e:
            print(f"❌ 오류 발생: {str(e)}")
            return {'error': "소스 디스펜싱 명령어를 인식하지 못했습니다."}
