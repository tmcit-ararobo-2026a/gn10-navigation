import sys
import zlib
import urllib.request
from pathlib import Path

# PlantUML encoding table
CHAR_MAP = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz-_"

def encode6bit(b):
    return CHAR_MAP[b & 0x3F]

def append3bytes(b1, b2, b3):
    c1 = b1 >> 2
    c2 = ((b1 & 0x3) << 4) | (b2 >> 4)
    c3 = ((b2 & 0xF) << 2) | (b3 >> 6)
    c4 = b3 & 0x3F
    return encode6bit(c1) + encode6bit(c2) + encode6bit(c3) + encode6bit(c4)

def plantuml_encode(text: str) -> str:
    compressed = zlib.compress(text.encode('utf-8'))[2:-4]
    res = []
    i = 0
    while i < len(compressed):
        b1 = compressed[i]
        b2 = compressed[i+1] if i+1 < len(compressed) else 0
        b3 = compressed[i+2] if i+2 < len(compressed) else 0
        res.append(append3bytes(b1, b2, b3))
        i += 3
    return "".join(res)

def render_pu_file(pu_path: Path):
    png_path = pu_path.with_suffix('.png')
    text = pu_path.read_text(encoding='utf-8')
    encoded = plantuml_encode(text)
    url = f"https://www.plantuml.com/plantuml/png/{encoded}"
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
    print(f"Downloading PNG for {pu_path.name} from PlantUML server...")
    with urllib.request.urlopen(req, timeout=15) as resp:
        data = resp.read()
    png_path.write_bytes(data)
    print(f"Saved: {png_path} ({len(data)} bytes)")

if __name__ == '__main__':
    for arg in sys.argv[1:]:
        p = Path(arg)
        if p.exists() and p.suffix == '.pu':
            render_pu_file(p)
