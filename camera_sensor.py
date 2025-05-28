#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RealSense 카메라 센서 모듈
깊이 이미지를 Isaac Lab 형식으로 제공
"""

import time
import threading
import numpy as np
import cv2
import pyrealsense2 as rs
from config import *


class CameraSensor:
    def __init__(self, width=CAMERA_WIDTH, height=CAMERA_HEIGHT, fps=CAMERA_FPS):
        """RealSense 카메라 센서 초기화"""
        try:
            self.width = width
            self.height = height
            self.fps = fps
            
            # RealSense pipeline 설정
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # 스트림 설정
            self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
            self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
            
            # 데이터 저장
            self.color_image = None
            self.depth_image = None              # 원본 (mm)
            self.depth_image_meters = None       # Isaac Lab 형식 (m)
            self.depth_colormap = None           # 시각화용
            
            # 스레드 안전성
            self.lock = threading.Lock()
            self.running = False
            self.update_thread = None
            
            if DEBUG_MODE:
                print(f"✅ 카메라 센서 초기화: {width}x{height}@{fps}Hz")
                
        except Exception as e:
            print(f"❌ 카메라 센서 초기화 실패: {e}")
            raise
    
    def start(self):
        """카메라 시작"""
        try:
            self.pipeline.start(self.config)
            self.running = True
            
            # 백그라운드 업데이트 스레드 시작
            self.update_thread = threading.Thread(target=self._update_thread, daemon=True)
            self.update_thread.start()
            
            if DEBUG_MODE:
                print("✅ 카메라 시작 완료")
            return True
            
        except Exception as e:
            print(f"❌ 카메라 시작 실패: {e}")
            return False
    
    def _update_thread(self):
        """백그라운드에서 프레임 지속 업데이트"""
        while self.running:
            self._update_frames()
            time.sleep(0.01)  # 100fps 대신 고정 간격
    
    def _update_frames(self):
        """프레임 업데이트 - 작동하는 버전과 동일하게 수정"""
        if not self.running:
            return False
        
        try:
            # 타임아웃 제거 - 작동하는 코드와 동일
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return False
            
            with self.lock:
                # 컬러 이미지
                self.color_image = np.asanyarray(color_frame.get_data())
                
                # 깊이 이미지 (원본)
                self.depth_image = np.asanyarray(depth_frame.get_data())
                
                # Isaac Lab 형식으로 변환 (mm → m)
                self.depth_image_meters = self._convert_depth_to_meters(self.depth_image)
                
                # 시각화용 컬러맵 - 작동하는 코드와 동일
                self.depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(self.depth_image, alpha=0.03), 
                    cv2.COLORMAP_JET
                )
            
            return True
            
        except Exception as e:
            if VERBOSE_LOGGING:
                print(f"카메라 프레임 업데이트 오류: {e}")
            return False
    
    def _convert_depth_to_meters(self, depth_mm):
        """깊이 이미지 단위 변환 및 정리"""
        if depth_mm is None:
            return np.zeros((self.height, self.width), dtype=np.float32)
        
        # mm → m 변환
        depth_m = depth_mm.astype(np.float32) * MM_TO_M
        
        # 무한대 및 NaN 값 처리
        depth_m[depth_mm == 0] = 0.0  # 측정 실패 영역
        depth_m[np.isinf(depth_m)] = 0.0
        depth_m[np.isnan(depth_m)] = 0.0
        
        # 최대 거리 제한 (10미터)
        depth_m = np.clip(depth_m, 0.0, 10.0)
        
        return depth_m
    
    def get_isaac_lab_data(self):
        """Isaac Lab 호환 형식으로 깊이 데이터 반환"""
        with self.lock:
            if self.depth_image_meters is not None:
                return {
                    'depth_image': self.depth_image_meters.copy(),  # (H, W) in meters
                    'color_image': self.color_image.copy() if self.color_image is not None else None
                }
            else:
                return {
                    'depth_image': np.zeros((self.height, self.width), dtype=np.float32),
                    'color_image': None
                }
    
    def get_depth_image_meters(self):
        """깊이 이미지 반환 (미터 단위)"""
        with self.lock:
            if self.depth_image_meters is not None:
                return self.depth_image_meters.copy()
            else:
                return np.zeros((self.height, self.width), dtype=np.float32)
    
    def get_depth_image_raw(self):
        """원본 깊이 이미지 반환 (mm 단위)"""
        with self.lock:
            if self.depth_image is not None:
                return self.depth_image.copy()
            else:
                return None
    
    def get_color_image(self):
        """컬러 이미지 반환"""
        with self.lock:
            if self.color_image is not None:
                return self.color_image.copy()
            else:
                return None
    
    def get_combined_image(self):
        """컬러+깊이 결합 이미지 반환 (시각화용)"""
        with self.lock:
            if self.color_image is not None and self.depth_colormap is not None:
                return np.hstack((self.color_image, self.depth_colormap))
            else:
                return None
    
    def get_depth_statistics(self):
        """깊이 이미지 통계 정보"""
        depth_m = self.get_depth_image_meters()
        valid_depths = depth_m[depth_m > 0]
        
        if len(valid_depths) > 0:
            return {
                'min_depth': float(np.min(valid_depths)),
                'max_depth': float(np.max(valid_depths)),
                'mean_depth': float(np.mean(valid_depths)),
                'valid_pixels': len(valid_depths),
                'total_pixels': depth_m.size
            }
        else:
            return {
                'min_depth': 0.0,
                'max_depth': 0.0,
                'mean_depth': 0.0,
                'valid_pixels': 0,
                'total_pixels': depth_m.size
            }
    
    def is_connected(self):
        """카메라 연결 상태 확인"""
        return self.running and self.update_thread and self.update_thread.is_alive()
    
    def stop(self):
        """카메라 중지"""
        self.running = False
        
        if self.update_thread and self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)
        
        try:
            if self.pipeline:
                self.pipeline.stop()
        except:
            pass
        
        if DEBUG_MODE:
            print("🔌 카메라 중지")


# 테스트 코드
if __name__ == "__main__":
    print("🧪 카메라 센서 테스트 시작...")
    
    try:
        camera = CameraSensor()
        
        if camera.start():
            print("\n📊 5초간 깊이 데이터 모니터링...")
            
            for i in range(5):
                data = camera.get_isaac_lab_data()
                stats = camera.get_depth_statistics()
                
                print(f"\nFrame {i+1}:")
                print(f"  깊이 이미지 크기: {data['depth_image'].shape}")
                print(f"  최소 깊이: {stats['min_depth']:.3f}m")
                print(f"  최대 깊이: {stats['max_depth']:.3f}m")
                print(f"  평균 깊이: {stats['mean_depth']:.3f}m")
                print(f"  유효 픽셀: {stats['valid_pixels']}/{stats['total_pixels']}")
                
                # 이미지 표시 (옵션)
                combined = camera.get_combined_image()
                if combined is not None:
                    cv2.imshow('Camera Test', combined)
                    cv2.waitKey(1)
                
                time.sleep(1)
            
            cv2.destroyAllWindows()
        else:
            print("❌ 카메라 시작 실패")
            
    except KeyboardInterrupt:
        print("테스트 중단")
    except Exception as e:
        print(f"테스트 오류: {e}")
    finally:
        if 'camera' in locals():
            camera.stop()
