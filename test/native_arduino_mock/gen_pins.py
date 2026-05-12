for letter in range(ord('A'), ord('I') + 1):
    for num in range(0, 15 + 1):
        print(f"#define P{chr(letter)}{num} {(letter - ord('A')) * 16 + num}")