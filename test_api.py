#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Test API to verify ml_details is sent"""

import requests
import json

API_BASE = "http://localhost:5000"

print("Testing API endpoints...")
print("=" * 60)

# Test 1: Start game
print("\n1. Starting game...")
try:
    response = requests.get(f"{API_BASE}/api/start")
    print(f"Status: {response.status_code}")
    print(f"Response: {response.json()}")
except Exception as e:
    print(f"Error: {e}")

# Test 2: Analyze
print("\n2. Analyzing missile...")
try:
    response = requests.get(f"{API_BASE}/api/analyze")
    print(f"Status: {response.status_code}")
    data = response.json()
    
    print(f"\nResponse keys: {data.keys()}")
    
    if 'identification' in data:
        ident = data['identification']
        print(f"\nIdentification keys: {ident.keys()}")
        print(f"Predicted type: {ident.get('predicted_type')}")
        print(f"Confidence: {ident.get('confidence')}")
        
        if 'ml_details' in ident:
            ml = ident['ml_details']
            print(f"\n✅ ml_details found!")
            print(f"ml_details keys: {ml.keys()}")
            print(f"n_estimators: {ml.get('n_estimators')}")
            print(f"Number of detailed_features: {len(ml.get('detailed_features', []))}")
            print(f"Number of tree_predictions: {len(ml.get('tree_predictions', []))}")
            
            # Print top 3 features
            if ml.get('detailed_features'):
                print(f"\nTop 3 features:")
                for i, feat in enumerate(ml['detailed_features'][:3]):
                    print(f"  {i+1}. {feat['name']}: {feat['importance']}% (raw: {feat['raw_value']})")
        else:
            print(f"\n❌ ml_details NOT found in identification!")
            print(f"Full identification: {json.dumps(ident, indent=2)}")
    
    if 'trajectory_params' in data:
        print(f"\n✅ trajectory_params found: {data['trajectory_params']}")
    
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()

print("\n" + "=" * 60)
