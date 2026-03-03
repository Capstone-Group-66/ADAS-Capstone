import tokenize
import io

with open('src/main.cpp', 'r') as f:
    text = f.read()

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
                    # skip next char
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
        
        # EOF line
        in_line_comment = False
        
        if 'int main(' in line and not in_block_comment and not in_string:
            print(f"{filename}: int main found on line {i+1}. Depth BEFORE line: {depth - (line.count('{') - line.count('}'))}")

check_c_braces('src/main.cpp')
