#!/usr/bin/env python3
"""
Simple test script to verify Ctrl+C handling works properly.
This should respond immediately to Ctrl+C without hanging.
"""

import asyncio
import time

async def main():
    print("🧪 Testing Ctrl+C handling...")
    print("Press Ctrl+C to stop (should respond immediately)")
    
    start_time = time.time()
    count = 0
    
    try:
        while True:
            await asyncio.sleep(0.1)
            count += 1
            
            # Print status every 10 iterations (every second)
            if count % 10 == 0:
                elapsed = time.time() - start_time
                print(f"⏱️  Running for {elapsed:.1f} seconds - Press Ctrl+C to stop")
                
    except KeyboardInterrupt:
        elapsed = time.time() - start_time
        print(f"\n✅ Ctrl+C caught successfully after {elapsed:.1f} seconds!")
        print("🎉 Ctrl+C handling is working properly")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")

if __name__ == "__main__":
    asyncio.run(main()) 