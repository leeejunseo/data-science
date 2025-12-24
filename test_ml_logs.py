#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ML 분석 로그 테스트 스크립트
"""

import requests
import json

print("\n" + "="*60)
print("ML 분석 API 테스트")
print("="*60 + "\n")

try:
    # API 호출
    print("📡 /api/analyze 호출 중...")
    response = requests.get('http://localhost:5000/api/analyze')
    
    if response.status_code == 200:
        data = response.json()
        
        print("\n✅ API 응답 성공\n")
        print("="*60)
        print("ML 예측 결과")
        print("="*60)
        
        ident = data.get('identification', {})
        print(f"\n🎯 예측 미사일: {ident.get('predicted_type')}")
        print(f"📊 신뢰도: {ident.get('confidence')}%")
        print(f"🔍 실제 미사일: {data.get('actual_missile')}")
        
        print(f"\n📈 확률 분포:")
        probs = ident.get('all_probabilities', {})
        for missile, prob in probs.items():
            bar = "█" * int(prob / 2)
            print(f"  {missile:10s}: {prob:5.1f}% {bar}")
        
        print(f"\n💡 분석 근거:")
        for reason in ident.get('reasons', []):
            print(f"  • {reason}")
        
        print(f"\n📊 주요 특징:")
        features = ident.get('features', {})
        for key, value in features.items():
            print(f"  • {key}: {value}")
        
        print(f"\n🔧 분석 방법: {ident.get('method')}")
        
        if data.get('graph_image'):
            img_size = len(data['graph_image'])
            print(f"\n📈 그래프 이미지: {img_size:,} bytes (base64)")
        
        print("\n" + "="*60)
        
        # 결과 판정
        predicted = ident.get('predicted_type')
        actual = data.get('actual_missile')
        if predicted == actual:
            print("✅ ML 예측 정확!")
        else:
            print(f"❌ ML 예측 오류: {predicted} != {actual}")
        
        print("="*60 + "\n")
        
    else:
        print(f"❌ API 오류: {response.status_code}")
        print(response.text)
        
except Exception as e:
    print(f"❌ 에러 발생: {e}")
    import traceback
    traceback.print_exc()
