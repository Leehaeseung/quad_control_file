#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Isaac Lab 변환기
센서 데이터를 Isaac Lab 입력 형식으로 변환
"""

import time
import numpy as np
from config import *


class IsaacConverter:
    def __init__(self):
        """Isaac Lab 변환기 초기화"""
        self.H = CAMERA_HEIGHT
        self.W = CAMERA_WIDTH
        
        if DEBUG_MODE:
            print(f"✅ Isaac Lab 변환기 초기화: {self.H}x{self.W}")
    
    def sensors_to_isaac_format(self, imu_data, motor_data, camera_data):
        """
        센서 데이터를 Isaac Lab 형식으로 변환
        
        Args:
            imu_data: IMU 센서 데이터
            motor_data: 모터 센서 데이터  
            camera_data: 카메라 센서 데이터
            
        Returns:
            dict: Isaac Lab 호환 형식의 데이터
        """
        isaac_data = {}
        
        # 1. Base 정보 (IMU)
        if imu_data:
            isaac_data.update({
                'base_linear_velocity': imu_data.get('base_linear_velocity', [0.0, 0.0, 0.0]),
                'base_angular_velocity': imu_data.get('base_angular_velocity', [0.0, 0.0, 0.0]),
                'projected_gravity': imu_data.get('projected_gravity', [0.0, 0.0, -1.0])
            })
        else:
            isaac_data.update({
                'base_linear_velocity': [0.0, 0.0, 0.0],
                'base_angular_velocity': [0.0, 0.0, 0.0],
                'projected_gravity': [0.0, 0.0, -1.0]
            })
        
        # 2. Joint 정보 (모터)
        if motor_data:
            isaac_data.update({
                'joint_positions': motor_data.get('joint_positions', [0.0] * 8),
                'joint_velocities': motor_data.get('joint_velocities', [0.0] * 8)
            })
        else:
            isaac_data.update({
                'joint_positions': [0.0] * 8,
                'joint_velocities': [0.0] * 8
            })
        
        # 3. 깊이 이미지 (카메라)
        if camera_data and camera_data.get('depth_image') is not None:
            isaac_data['depth_image'] = camera_data['depth_image']
        else:
            isaac_data['depth_image'] = np.zeros((self.H, self.W), dtype=np.float32)
        
        return isaac_data
    
    def create_base_channel(self, isaac_data):
        """
        Base 정보를 3x3 격자로 배치한 채널 생성
        
        Returns:
            np.array: (H, W) 형태의 base 채널
        """
        # 9개 값 결합
        base_values = (
            isaac_data['base_linear_velocity'] +     # [0, 1, 2]
            isaac_data['base_angular_velocity'] +    # [3, 4, 5]  
            isaac_data['projected_gravity']          # [6, 7, 8]
        )
        
        # 3x3 격자 배치
        base_channel = np.zeros((self.H, self.W), dtype=np.float32)
        
        ph, pw = self.H // 3, self.W // 3
        
        for idx in range(9):
            i, j = divmod(idx, 3)  # 3x3 격자 좌표
            value = base_values[idx] if idx < len(base_values) else 0.0
            
            # 패치 좌표 계산
            r_start, r_end = i * ph, (i + 1) * ph
            c_start, c_end = j * pw, (j + 1) * pw
            
            # 마지막 패치 경계 조정
            if i == 2: r_end = self.H
            if j == 2: c_end = self.W
            
            # 해당 영역에 값 채우기
            base_channel[r_start:r_end, c_start:c_end] = value
        
        return base_channel
    
    def create_joint_channel(self, isaac_data):
        """
        Joint 정보를 4개 사분면으로 배치한 채널 생성
        
        Returns:
            np.array: (H, W) 형태의 joint 채널
        """
        joint_positions = isaac_data['joint_positions']
        joint_velocities = isaac_data['joint_velocities']
        
        joint_channel = np.zeros((self.H, self.W), dtype=np.float32)
        
        # 사분면 크기 계산
        ph2, pw2 = self.H // 2, self.W // 2
        phq, pwq = ph2 // 2, pw2 // 2
        
        # 4개 사분면 배치 (Isaac Lab 조인트 순서)
        # 조인트 쌍: (0,1), (2,3), (4,5), (6,7)
        joint_pairs = [(0, 1), (2, 3), (4, 5), (6, 7)]
        
        # 사분면 좌표
        quadrant_coords = [
            (0, ph2, 0, pw2),           # 좌상 (조인트 0,1)
            (0, ph2, pw2, self.W),      # 우상 (조인트 2,3)
            (ph2, self.H, 0, pw2),      # 좌하 (조인트 4,5)
            (ph2, self.H, pw2, self.W)  # 우하 (조인트 6,7)
        ]
        
        for quad_idx, ((j0, j1), (r0, r1, c0, c1)) in enumerate(zip(joint_pairs, quadrant_coords)):
            # 각 사분면을 4개 구역으로 분할
            # 좌상: joint position, 우상: joint velocity
            # 좌하: next joint position, 우하: next joint velocity
            
            pos0 = joint_positions[j0] if j0 < len(joint_positions) else 0.0
            vel0 = joint_velocities[j0] if j0 < len(joint_velocities) else 0.0
            pos1 = joint_positions[j1] if j1 < len(joint_positions) else 0.0
            vel1 = joint_velocities[j1] if j1 < len(joint_velocities) else 0.0
            
            # 좌상: joint0 position
            joint_channel[r0:r0+phq, c0:c0+pwq] = pos0
            
            # 우상: joint0 velocity
            joint_channel[r0:r0+phq, c0+pwq:c1] = vel0
            
            # 좌하: joint1 position  
            joint_channel[r0+phq:r1, c0:c0+pwq] = pos1
            
            # 우하: joint1 velocity
            joint_channel[r0+phq:r1, c0+pwq:c1] = vel1
        
        return joint_channel
    
    def create_isaac_tensor(self, isaac_data):
        """
        Isaac Lab 형식의 (H, W, 3) 텐서 생성
        
        Returns:
            np.array: (H, W, 3) 형태의 Isaac Lab 입력 텐서
        """
        # 채널 0: 깊이 이미지
        depth_channel = isaac_data['depth_image']
        
        # 채널 1: Base 정보 (3x3 격자)
        base_channel = self.create_base_channel(isaac_data)
        
        # 채널 2: Joint 정보 (4개 사분면)
        joint_channel = self.create_joint_channel(isaac_data)
        
        # 3채널 결합
        isaac_tensor = np.stack([depth_channel, base_channel, joint_channel], axis=-1)
        
        return isaac_tensor
    
    def format_isaac_values_string(self, isaac_data):
        """
        Isaac Lab 형식의 값들을 한 줄 문자열로 포맷
        
        Returns:
            str: Isaac Lab 형식 출력 문자열
        """
        lin_vel = isaac_data['base_linear_velocity']
        ang_vel = isaac_data['base_angular_velocity']
        gravity = isaac_data['projected_gravity']
        joint_pos = isaac_data['joint_positions']
        joint_vel = isaac_data['joint_velocities']
        
        output_line = (
            f"linx: {lin_vel[0]:.4f} liny: {lin_vel[1]:.4f} linz: {lin_vel[2]:.4f} "
            f"angx: {ang_vel[0]:.4f} angy: {ang_vel[1]:.4f} angz: {ang_vel[2]:.4f} "
            f"gravx: {gravity[0]:.4f} gravy: {gravity[1]:.4f} gravz: {gravity[2]:.4f} "
            f"jp0: {joint_pos[0]:.4f} jp1: {joint_pos[1]:.4f} jp2: {joint_pos[2]:.4f} jp3: {joint_pos[3]:.4f} "
            f"jp4: {joint_pos[4]:.4f} jp5: {joint_pos[5]:.4f} jp6: {joint_pos[6]:.4f} jp7: {joint_pos[7]:.4f} "
            f"jv0: {joint_vel[0]:.4f} jv1: {joint_vel[1]:.4f} jv2: {joint_vel[2]:.4f} jv3: {joint_vel[3]:.4f} "
            f"jv4: {joint_vel[4]:.4f} jv5: {joint_vel[5]:.4f} jv6: {joint_vel[6]:.4f} jv7: {joint_vel[7]:.4f}"
        )
        
        return output_line
    
    def validate_isaac_data(self, isaac_data):
        """
        Isaac Lab 데이터 유효성 검사
        
        Returns:
            bool: 데이터가 유효한지 여부
        """
        required_keys = [
            'base_linear_velocity', 'base_angular_velocity', 'projected_gravity',
            'joint_positions', 'joint_velocities', 'depth_image'
        ]
        
        for key in required_keys:
            if key not in isaac_data:
                if VERBOSE_LOGGING:
                    print(f"❌ 누락된 키: {key}")
                return False
        
        # 배열 크기 검사
        if len(isaac_data['base_linear_velocity']) != 3:
            return False
        if len(isaac_data['base_angular_velocity']) != 3:
            return False
        if len(isaac_data['projected_gravity']) != 3:
            return False
        if len(isaac_data['joint_positions']) != 8:
            return False
        if len(isaac_data['joint_velocities']) != 8:
            return False
        
        # 깊이 이미지 크기 검사
        depth_shape = isaac_data['depth_image'].shape
        if depth_shape != (self.H, self.W):
            if VERBOSE_LOGGING:
                print(f"❌ 깊이 이미지 크기 오류: {depth_shape}, 기대값: ({self.H}, {self.W})")
            return False
        
        return True
    
    def get_tensor_info(self, tensor):
        """텐서 정보 반환"""
        return {
            'shape': tensor.shape,
            'dtype': tensor.dtype,
            'depth_range': (tensor[:,:,0].min(), tensor[:,:,0].max()),
            'base_range': (tensor[:,:,1].min(), tensor[:,:,1].max()),
            'joint_range': (tensor[:,:,2].min(), tensor[:,:,2].max())
        }


# 테스트 코드
if __name__ == "__main__":
    print("🧪 Isaac Lab 변환기 테스트 시작...")
    
    # 더미 데이터 생성
    dummy_imu = {
        'base_linear_velocity': [0.1, 0.05, -0.02],
        'base_angular_velocity': [0.2, -0.1, 0.05],
        'projected_gravity': [0.1, 0.05, -0.99]
    }
    
    dummy_motor = {
        'joint_positions': [0.1, 0.2, -0.1, 0.15, 0.05, -0.05, 0.3, -0.2],
        'joint_velocities': [1.0, -0.5, 0.8, -1.2, 0.3, 0.7, -0.9, 1.1]
    }
    
    dummy_camera = {
        'depth_image': np.random.rand(CAMERA_HEIGHT, CAMERA_WIDTH) * 3.0  # 0-3m 랜덤 깊이
    }
    
    try:
        converter = IsaacConverter()
        
        # 변환 테스트
        isaac_data = converter.sensors_to_isaac_format(dummy_imu, dummy_motor, dummy_camera)
        
        # 유효성 검사
        if converter.validate_isaac_data(isaac_data):
            print("✅ 데이터 유효성 검사 통과")
        
        # 텐서 생성
        tensor = converter.create_isaac_tensor(isaac_data)
        info = converter.get_tensor_info(tensor)
        
        print(f"\n📊 생성된 Isaac Lab 텐서:")
        print(f"  형태: {info['shape']}")
        print(f"  데이터 타입: {info['dtype']}") 
        print(f"  깊이 채널 범위: {info['depth_range'][0]:.3f} ~ {info['depth_range'][1]:.3f}")
        print(f"  베이스 채널 범위: {info['base_range'][0]:.3f} ~ {info['base_range'][1]:.3f}")
        print(f"  조인트 채널 범위: {info['joint_range'][0]:.3f} ~ {info['joint_range'][1]:.3f}")
        
        # 문자열 출력
        print(f"\n📝 Isaac Lab 형식 출력:")
        print(converter.format_isaac_values_string(isaac_data))
        
    except Exception as e:
        print(f"테스트 오류: {e}")
        import traceback
        traceback.print_exc()