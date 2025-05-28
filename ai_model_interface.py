#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AI 모델 인터페이스
Isaac Lab 학습된 모델을 로드하고 실행하는 인터페이스
"""

import time
import numpy as np
import torch
from config import *


class AIModelInterface:
    def __init__(self, model_path=None):
        """AI 모델 인터페이스 초기화"""
        print("🤖 AI 모델 인터페이스 초기화...")
        
        self.model_path = model_path
        self.model = None
        self.device = None
        self.is_loaded = False
        
        # 입출력 형태
        self.input_shape = (CAMERA_HEIGHT, CAMERA_WIDTH, 3)  # Isaac Lab 입력 형태
        self.output_dim = 16  # 16개 출력 (상위 8개만 사용)
        
        # 성능 모니터링
        self.inference_count = 0
        self.total_inference_time = 0.0
        self.last_inference_time = 0.0
        
        # GPU/CPU 설정
        self._setup_device()
        
        if DEBUG_MODE:
            print(f"✅ AI 모델 인터페이스 초기화 완료")
            print(f"   장치: {self.device}")
            print(f"   입력 형태: {self.input_shape}")
            print(f"   출력 차원: {self.output_dim}")
    
    def _setup_device(self):
        """GPU/CPU 장치 설정"""
        if torch.cuda.is_available():
            self.device = torch.device('cuda')
            if DEBUG_MODE:
                print(f"🚀 CUDA 사용 가능: {torch.cuda.get_device_name()}")
        else:
            self.device = torch.device('cpu')
            if DEBUG_MODE:
                print("💻 CPU 사용")
    
    def load_model(self, model_path):
        """AI 모델 로드"""
        if not model_path:
            print("❌ 모델 경로가 지정되지 않음")
            return False
        
        try:
            print(f"📂 AI 모델 로딩 중: {model_path}")
            
            # PyTorch 모델 로드 (Isaac Lab에서 학습된 모델)
            self.model = torch.load(model_path, map_location=self.device)
            
            # 모델을 평가 모드로 설정
            if hasattr(self.model, 'eval'):
                self.model.eval()
            
            # 추론 모드 설정 (그래디언트 계산 비활성화)
            torch.set_grad_enabled(False)
            
            self.model_path = model_path
            self.is_loaded = True
            
            print(f"✅ AI 모델 로드 완료")
            
            # 모델 정보 출력
            if DEBUG_MODE:
                self._print_model_info()
            
            return True
            
        except Exception as e:
            print(f"❌ AI 모델 로드 실패: {e}")
            self.model = None
            self.is_loaded = False
            return False
    
    def _print_model_info(self):
        """모델 정보 출력"""
        if self.model is None:
            return
        
        try:
            # 모델 파라미터 수 계산
            if hasattr(self.model, 'parameters'):
                total_params = sum(p.numel() for p in self.model.parameters())
                trainable_params = sum(p.numel() for p in self.model.parameters() if p.requires_grad)
            else:
                total_params = 0
                trainable_params = 0
            
            print(f"📊 모델 정보:")
            print(f"   파일: {self.model_path}")
            print(f"   전체 매개변수: {total_params:,}")
            print(f"   훈련 가능 매개변수: {trainable_params:,}")
            print(f"   모델 타입: {type(self.model).__name__}")
            print(f"   장치: {self.device}")
            
            # 테스트 추론으로 입출력 확인 - 수정된 부분
            try:
                # Isaac Lab 형식: (H, W, 3) → PyTorch: (1, 3, H, W)
                test_input = torch.zeros(1, 3, CAMERA_HEIGHT, CAMERA_WIDTH, dtype=torch.float32).to(self.device)
                with torch.no_grad():
                    test_output = self.model(test_input)
                    output_shape = test_output.shape
                    
                print(f"   입력 형태: (1, 3, {CAMERA_HEIGHT}, {CAMERA_WIDTH})")
                print(f"   출력 형태: {output_shape}")
                
                if len(output_shape) >= 2 and output_shape[1] >= 8:
                    print(f"   ✅ 조인트 출력 확인: {output_shape[1]}개 (상위 8개 사용)")
                else:
                    print(f"   ⚠️ 출력 형태 확인 필요: {output_shape}")
                    
            except Exception as e:
                print(f"   ⚠️ 테스트 추론 실패: {e}")
                
        except Exception as e:
            print(f"❌ 모델 정보 출력 오류: {e}")
    
    def predict(self, isaac_tensor):
        """
        Isaac Lab 텐서를 입력받아 조인트 각도 예측
        
        Args:
            isaac_tensor: (H, W, 3) 형태의 Isaac Lab 입력 텐서
            
        Returns:
            dict: 예측 결과
                - 'isaac_joint_angles': Isaac Lab 조인트 순서 (8개)
                - 'success': 예측 성공 여부
                - 'inference_time': 추론 시간
        """
        if not self.is_loaded:
            print("❌ 모델이 로드되지 않음")
            return {'isaac_joint_angles': np.zeros(8), 'success': False, 'inference_time': 0.0}
        
        try:
            start_time = time.time()
            
            # 입력 형태 검증
            if isinstance(isaac_tensor, np.ndarray):
                if isaac_tensor.shape != (CAMERA_HEIGHT, CAMERA_WIDTH, 3):
                    print(f"❌ 입력 형태 오류: {isaac_tensor.shape}, 기대값: ({CAMERA_HEIGHT}, {CAMERA_WIDTH}, 3)")
                    return {'isaac_joint_angles': np.zeros(8), 'success': False, 'inference_time': 0.0}
                
                # NumPy → PyTorch 텐서 변환: (H, W, 3) → (1, 3, H, W)
                input_tensor = torch.from_numpy(isaac_tensor).float()
                input_tensor = input_tensor.permute(2, 0, 1)  # (H, W, 3) → (3, H, W)
                input_tensor = input_tensor.unsqueeze(0)      # (3, H, W) → (1, 3, H, W)
            else:
                # 이미 텐서인 경우
                input_tensor = isaac_tensor.float()
                if len(input_tensor.shape) == 3:  # (H, W, 3)
                    input_tensor = input_tensor.permute(2, 0, 1).unsqueeze(0)  # → (1, 3, H, W)
                elif len(input_tensor.shape) == 4 and input_tensor.shape[0] == 1:  # (1, H, W, 3)
                    input_tensor = input_tensor.squeeze(0).permute(2, 0, 1).unsqueeze(0)  # → (1, 3, H, W)
            
            input_tensor = input_tensor.to(self.device)
            
            # 추론 실행
            with torch.no_grad():
                output = self.model(input_tensor)
                
                # CPU로 이동 및 NumPy 변환
                if isinstance(output, torch.Tensor):
                    output = output.cpu().numpy()
                
                # 배치 차원 제거
                if len(output.shape) > 1 and output.shape[0] == 1:
                    output = output[0]
            
            # 상위 8개 조인트 각도 추출
            if len(output) >= 8:
                isaac_joint_angles = output[:8]
            else:
                print(f"⚠️ 출력 크기 부족: {len(output)}, 8개 필요")
                isaac_joint_angles = np.zeros(8)
            
            # 성능 측정
            inference_time = time.time() - start_time
            self.inference_count += 1
            self.total_inference_time += inference_time
            self.last_inference_time = inference_time
            
            if VERBOSE_LOGGING:
                print(f"🔮 AI 추론 완료: {inference_time*1000:.1f}ms")
                print(f"   조인트 각도 (rad): {isaac_joint_angles}")
            
            return {
                'isaac_joint_angles': isaac_joint_angles,
                'success': True,
                'inference_time': inference_time
            }
            
        except Exception as e:
            print(f"❌ AI 추론 실패: {e}")
            import traceback
            traceback.print_exc()
            return {
                'isaac_joint_angles': np.zeros(8),
                'success': False,
                'inference_time': 0.0
            }
    def predict_to_dynamixel_mapping(self, isaac_tensor):
        """
        Isaac Lab 텐서를 입력받아 Dynamixel ID 순서로 조인트 각도 반환
        
        Returns:
            dict: Dynamixel 매핑된 결과
                - 'dynamixel_angles': Dynamixel ID 순서 조인트 각도
                - 'isaac_angles': Isaac Lab 순서 조인트 각도  
                - 'success': 예측 성공 여부
        """
        # AI 모델 추론
        result = self.predict(isaac_tensor)
        
        if not result['success']:
            return {
                'dynamixel_angles': np.zeros(8),
                'isaac_angles': np.zeros(8),
                'success': False
            }
        
        isaac_angles = result['isaac_joint_angles']
        
        # Isaac Lab → Dynamixel ID 매핑
        dynamixel_angles = np.zeros(8)
        for isaac_idx in range(8):
            dynamixel_angles[isaac_idx] = isaac_angles[isaac_idx]
        
        return {
            'dynamixel_angles': dynamixel_angles,
            'isaac_angles': isaac_angles,
            'success': True
        }
    
    def get_performance_stats(self):
        """성능 통계 반환"""
        if self.inference_count == 0:
            return {
                'total_inferences': 0,
                'average_time': 0.0,
                'last_time': 0.0,
                'fps': 0.0
            }
        
        avg_time = self.total_inference_time / self.inference_count
        fps = 1.0 / avg_time if avg_time > 0 else 0.0
        
        return {
            'total_inferences': self.inference_count,
            'average_time': avg_time,
            'last_time': self.last_inference_time,
            'fps': fps
        }
    
    def reset_performance_stats(self):
        """성능 통계 초기화"""
        self.inference_count = 0
        self.total_inference_time = 0.0
        self.last_inference_time = 0.0
    
    def is_model_loaded(self):
        """모델 로드 상태 확인"""
        return self.is_loaded and self.model is not None
    
    def unload_model(self):
        """모델 언로드"""
        if self.model is not None:
            del self.model
            self.model = None
            self.is_loaded = False
            
            # GPU 메모리 정리
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
            
            print("🗑️ AI 모델 언로드 완료")


# 테스트 코드
if __name__ == "__main__":
    print("🧪 AI 모델 인터페이스 테스트 시작...")
    
    try:
        # AI 모델 인터페이스 초기화
        ai_interface = AIModelInterface()
        
        # 더미 Isaac Lab 텐서 생성
        dummy_tensor = np.random.rand(CAMERA_HEIGHT, CAMERA_WIDTH, 3).astype(np.float32)
        
        print(f"\n📊 더미 텐서 생성:")
        print(f"   형태: {dummy_tensor.shape}")
        print(f"   데이터 타입: {dummy_tensor.dtype}")
        print(f"   범위: {dummy_tensor.min():.3f} ~ {dummy_tensor.max():.3f}")
        
        # 모델 없이 테스트 (실패 예상)
        print(f"\n🔮 모델 없이 추론 테스트:")
        result = ai_interface.predict(dummy_tensor)
        print(f"   성공: {result['success']}")
        print(f"   조인트 각도: {result['isaac_joint_angles']}")
        
        # 성능 통계 확인
        stats = ai_interface.get_performance_stats()
        print(f"\n📈 성능 통계:")
        print(f"   총 추론 횟수: {stats['total_inferences']}")
        print(f"   평균 시간: {stats['average_time']*1000:.1f}ms")
        print(f"   예상 FPS: {stats['fps']:.1f}")
        
        print("\n✅ AI 모델 인터페이스 테스트 완료")
        
    except Exception as e:
        print(f"❌ 테스트 오류: {e}")
        import traceback
        traceback.print_exc()