"""
Test script to verify button command mappings
"""

def test_lambda_closure():
    """Test if lambda closures work correctly"""
    
    commands = [
        ("Get Angles", "."),
        ("Reset Yaw", "z"),
        ("Raw Data", "r"),
        ("Heading", "h"),
        ("🧭 Cal Mag", "m"),
        ("🔄 Cal Gyro", "c")
    ]
    
    # Test the lambda closure issue
    buttons = []
    
    print("Testing lambda closures:")
    
    # Original approach (might have closure issue)
    for i, (text, cmd) in enumerate(commands):
        btn_func = lambda c=cmd: print(f"Button '{text}' sends command: '{c}'")
        buttons.append((text, btn_func))
    
    # Test all buttons
    for text, func in buttons:
        func()
    
    print("\nExpected commands:")
    for text, cmd in commands:
        print(f"Button '{text}' should send: '{cmd}'")

if __name__ == "__main__":
    test_lambda_closure()
