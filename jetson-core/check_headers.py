import os
import sys

def check_c_braces(filename):
    with open(filename, 'r', encoding='utf-8') as f:
        content = f.read()
    
    depth = 0
    in_string = False
    in_char = False
    in_line_comment = False
    in_block_comment = False
    escape = False
    
    for i, line in enumerate(content.split('\n')):
        for j, c in enumerate(line):
            if in_line_comment:
                continue
            if in_block_comment:
                if c == '*' and j+1 < len(line) and line[j+1] == '/':
                    in_block_comment = False
                continue
            
            if in_string:
                if escape: escape = False
                elif c == '\\': escape = True
                elif c == '"': in_string = False
                continue
                
            if in_char:
                if escape: escape = False
                elif c == '\\': escape = True
                elif c == "'": in_char = False
                continue
                
            if c == '/' and j+1 < len(line):
                if line[j+1] == '/':
                    in_line_comment = True
                    continue
                elif line[j+1] == '*':
                    in_block_comment = True
                    continue
                    
            if c == '"':
                in_string = True
            elif c == "'":
                in_char = True
            elif c == '{':
                depth += 1
            elif c == '}':
                depth -= 1
        
        in_line_comment = False
        
    if depth != 0:
        print(f"UNBALANCED: {filename} has EOF depth = {depth}")
        return True
    return False

found_any = False
for root, _, files in os.walk('include'):
    for file in files:
        if file.endswith('.hpp'):
            if check_c_braces(os.path.join(root, file)):
                found_any = True

if not found_any:
    print("All headers are perfectly balanced.")
