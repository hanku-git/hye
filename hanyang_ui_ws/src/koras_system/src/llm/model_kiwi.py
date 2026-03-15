# import whisper
# from kiwipiepy import Kiwi
# import textdistance
# import re

# from llm_node.model_llm_cook import LLMModel


# # STTModel: 음성을 텍스트로 변환
# class STTModel:
#     def __init__(self, filepath):
#         self.stt_model = whisper.load_model("small", in_memory=True)
#         self.stt_options = whisper.DecodingOptions(language="Korean")
#         self.filepath = filepath

#     def get_text(self):
#         audio = whisper.load_audio(self.filepath)
#         audio = whisper.pad_or_trim(audio)
#         mel = whisper.log_mel_spectrogram(audio).to(self.stt_model.device)
#         stt_result = whisper.decode(self.stt_model, mel, self.stt_options)
#         return stt_result.text

# # BaseTask: 텍스트 전처리 및 오타 수정
# class BaseTask:
#     def __init__(self, similarity_threshold=0.7):
#         self.kiwi = Kiwi()
#         self.similarity_threshold = similarity_threshold
#         self.custom_words = [
#             '케첩', '캐첩소스', '캐첩', '갯처', '치폴레', '치뽈레', '치풀래', '치폴래', '갈릭치즈', '갈릭', 'Ketchup', 'Garlic Cheese', 'Chipotle', 'Guacamole', '치즈', '초록', '과카몰리', 'ketchup', 'garlic cheese', 'chipotle', 'guacamole', '과카', '' '소스', '담아', '넣어', '줘', '추가', '빼', '담아줘', '넣어줘'
#         ]
#         self.add_custom_words()

#     def add_custom_words(self):
#         for word in self.custom_words:
#             self.kiwi.add_user_word(word, 'NNG')

#     def correct_typo(self, word):
#         closest_match = None
#         highest_similarity = 0

#         for custom_word in self.custom_words:
#             similarity = textdistance.jaro_winkler(word, custom_word)
#             if similarity > highest_similarity:
#                 closest_match = custom_word
#                 highest_similarity = similarity

#         return closest_match if highest_similarity > self.similarity_threshold else word

#     def split_text(self, text):
#         pattern = '|'.join(map(re.escape, self.custom_words))
#         split_text = re.split(f'({pattern})', text)
#         split_text = [word for word in split_text if word.strip()]
#         return split_text

# # SourceTask: 소스 디스펜싱 명령어 처리
# class SourceTask(BaseTask):
#     def __init__(self, similarity_threshold=0.7):
#         super().__init__(similarity_threshold)
#         self.source_synonyms = {
#             'Ketchup': ['케첩', '케찹', '개찹', '갯처', 'Ketchup', 'ketchup'],
#             'Chipotle': ['치폴레', '치뽈레', 'Chipotle', 'chipotle', '치뽈레', '치풀래'],
#             'Garlic Cheese': ['갈릭치즈', '갈릭', '치즈', 'Garlic Cheese', 'garlic cheese'],
#             'Guacamole': ['과카몰리', '과카', '몰리', 'guacamole', 'Guacamole', '과카월리']
#         }

#     def map_word_to_synonym(self, word):
#         for key, synonyms in self.source_synonyms.items():
#             if word in synonyms:
#                 return key
#         return None

#     def process_source_command(self, text):
#         final_words = self.split_text(text)
#         source_type = None

#         for word in final_words:
#             mapped_source = self.map_word_to_synonym(word)
#             if mapped_source:
#                 source_type = mapped_source
#                 break

#         if source_type:
#             return {'task': 'dispense', 'source': source_type}

#         return {'error': "소스를 인식하지 못했습니다. 다시 시도해주세요."}

# # KIWIModel: 전체 명령어 처리
# class KIWIModel(BaseTask):
#     def __init__(self):
#         self.LLM = LLMModel()
#         self.source_task = SourceTask(similarity_threshold=0.7)

#     def get_result(self, text):
#         source_result = self.source_task.process_source_command(text)
#         if 'task' in source_result:
#             return source_result  # 띄어쓰기 제거하지 않음
#         elif 'error' in source_result:
#             return source_result

#         llm_output = self.LLM.get_result('error', text)
#         return {'error': llm_output}


import whisper
from kiwipiepy import Kiwi
import textdistance
import re

from llm_node.model_llm_cook import LLMModel

# STTModel: 음성을 텍스트로 변환 (Whisper를 활용한 robust 구현)
class STTModel:
    def __init__(self, filepath):
        # Whisper 모델을 메모리 내에서 로드하여 빠른 추론을 지원합니다.
        self.stt_model = whisper.load_model("small", in_memory=True)
        # 한국어 인식을 위해 Decoding 옵션 설정
        self.stt_options = whisper.DecodingOptions(language="Korean")
        self.filepath = filepath

    def get_text(self):
        # 오디오 파일 로드 및 전처리
        audio = whisper.load_audio(self.filepath)
        audio = whisper.pad_or_trim(audio)
        mel = whisper.log_mel_spectrogram(audio).to(self.stt_model.device)
        stt_result = whisper.decode(self.stt_model, mel, self.stt_options)
        return stt_result.text.strip()

# BaseTask: 텍스트 전처리, 토큰화 및 오타 수정 기능 제공
class BaseTask:
    def __init__(self, similarity_threshold=0.7):
        self.kiwi = Kiwi()
        self.similarity_threshold = similarity_threshold
        # 다양한 오타 및 발음 변형을 고려한 사용자 정의 단어 목록 (중복 제거)
        self.custom_words = list(set([
            '케첩', '케찹', '캐첩', '갯처', '카첩', '카챱',
            '캐첩소스',
            '치폴레', '치뽈레', '치폴래', '치퐁', '치풀래',
            '갈릭치즈', '갈릭', '갈릭치즈소스',
            'Guacamole', '과카몰리', '과카', '과꼬몰리', '구아카몰리',
            'Chipotle', 'Ketchup', 'Garlic Cheese', 'cheese',
            '소스', '담아', '넣어', '줘', '추가', '빼', '담아줘', '넣어줘', '추가해', '추가해주세요'
        ]))
        self.add_custom_words()

    def add_custom_words(self):
        # Kiwi 사용자 사전에 단어들을 추가하여 토큰화 정확도를 향상시킵니다.
        for word in self.custom_words:
            self.kiwi.add_user_word(word, 'NNG')

    def correct_typo(self, word):
        """
        입력 단어와 custom_words 목록의 각 단어 간 Jaro-Winkler 유사도를 계산하여,
        임계치(similarity_threshold) 이상이면 가장 유사한 단어로 보정합니다.
        """
        closest_match = None
        highest_similarity = 0
        for custom_word in self.custom_words:
            similarity = textdistance.jaro_winkler(word.lower(), custom_word.lower())
            if similarity > highest_similarity:
                closest_match = custom_word
                highest_similarity = similarity
        return closest_match if highest_similarity > self.similarity_threshold else word

    def preprocess_text(self, text):
        """
        1. 텍스트를 소문자로 변환하고 좌우 공백 제거.
        2. Kiwi를 활용하여 토큰화한 후, 각 토큰에서 불필요한 기호 제거.
        3. 각 토큰에 대해 오타 보정을 적용하여 최종 토큰 리스트 반환.
        """
        text = text.lower().strip()
        tokens = self.kiwi.tokenize(text)
        processed_tokens = []
        for token in tokens:
            token_str = token.form if hasattr(token, 'form') else str(token)
            token_str = re.sub(r'[^\w가-힣]', '', token_str)
            if token_str:
                corrected = self.correct_typo(token_str)
                processed_tokens.append(corrected)
        return processed_tokens

    def split_text(self, text):
        """
        Kiwi 기반 토큰화가 어려운 경우 대비하여, custom_words를 기준으로 정규표현식 분할을 수행합니다.
        분할된 각 토큰에 대해 오타 보정을 적용합니다.
        """
        pattern = '|'.join(map(re.escape, self.custom_words))
        split_tokens = re.split(f'({pattern})', text)
        split_tokens = [token for token in split_tokens if token.strip()]
        corrected_tokens = [self.correct_typo(token.lower().strip()) for token in split_tokens]
        return corrected_tokens

# SourceTask: 소스 디스펜싱 명령어를 처리 (오타 및 다양한 표현 고려, Fuzzy Matching 적용)
class SourceTask(BaseTask):
    def __init__(self, similarity_threshold=0.7):
        super().__init__(similarity_threshold)
        self.source_synonyms = {
            'Ketchup': ['케첩', '케찹', '캐첩', '갯처', '카첩', '카챱', 'ketchup'],
            'Chipotle': ['치폴레', '치뽈레', '치폴래', '치퐁', '치풀래', 'chipotle'],
            'Garlic Cheese': ['갈릭치즈', '갈릭', '갈릭치즈소스', 'garlic cheese'],
            'Guacamole': ['과카몰리', '과카', '과꼬몰리', '구아카몰리', 'guacamole']
        }
        # 동의어들을 소문자로 정규화
        for key, synonyms in self.source_synonyms.items():
            self.source_synonyms[key] = [syn.lower() for syn in synonyms]

    def map_word_to_synonym(self, word):
        """
        입력 단어와 각 소스 동의어 간의 유사도를 계산하여,
        가장 높은 유사도가 임계치(threshold) 이상이면 해당 소스 이름(key)를 반환합니다.
        """
        word_lower = word.lower()
        best_match = None
        best_similarity = 0.0
        threshold = 0.75  # 유사도 임계치 (필요에 따라 조정)
        for key, synonyms in self.source_synonyms.items():
            for syn in synonyms:
                similarity = textdistance.jaro_winkler(word_lower, syn)
                if similarity > best_similarity:
                    best_similarity = similarity
                    best_match = key
        if best_similarity >= threshold:
            return best_match
        return None

    def process_source_command(self, text):
        """
        1. Kiwi 기반 전처리(preprocess_text)를 통해 토큰화 및 보정을 수행합니다.
        2. 각 토큰에 대해 소스 동의어 매핑을 시도합니다.
        3. 만약 Kiwi 기반 전처리로 인식이 어려운 경우, split_text()를 통한 보조 처리 수행.
        4. 인식 실패 시 에러 JSON을 반환합니다.
        """
        tokens = self.preprocess_text(text)
        source_type = None
        for token in tokens:
            mapped_source = self.map_word_to_synonym(token)
            if mapped_source:
                source_type = mapped_source
                break
        if not source_type:
            fallback_tokens = self.split_text(text)
            for token in fallback_tokens:
                mapped_source = self.map_word_to_synonym(token)
                if mapped_source:
                    source_type = mapped_source
                    break
        if source_type:
            return {'task': 'dispense', 'source': source_type}
        return {'error': "소스를 인식하지 못했습니다. 다시 시도해주세요."}

# KIWIModel: 전체 명령어 처리 (음성 인식 후 최종 명령어 도출)
class KIWIModel(BaseTask):
    def __init__(self):
        super().__init__()
        self.LLM = LLMModel()
        self.source_task = SourceTask(similarity_threshold=0.7)

    def get_result(self, text):
        """
        1. robust하게 전처리된 텍스트를 기반으로 SourceTask로 소스 명령어를 도출합니다.
        2. 유효한 소스 명령어가 도출되면 해당 결과를 반환합니다.
        3. 인식 실패 시 LLM fallback으로 에러 처리를 진행합니다.
        """
        source_result = self.source_task.process_source_command(text)
        if 'task' in source_result:
            return source_result
        elif 'error' in source_result:
            return source_result
        llm_output = self.LLM.get_result('error', text)
        return {'error': llm_output}
