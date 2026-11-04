"""Bake the small UI character set; runtime needs no font packages/Python."""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
import re
root=Path(__file__).resolve().parent.parent
font_path='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font=ImageFont.truetype(font_path,48,index=2)  # Simplified Chinese
source=(root/'src/dashboard.c').read_text()
chars=sorted(set(chr(i) for i in range(32,127))|set(re.findall(r'[^\x00-\x7f]',source)),key=ord)
all_pixels=bytearray(); glyphs=[]
for ch in chars:
    im=Image.new('L',(64,64)); ImageDraw.Draw(im).text((2,52),ch,font=font,fill=255,anchor='ls')
    all_pixels.extend(im.tobytes()); glyphs.append((ord(ch),font.getlength(ch)))
rle=[]; i=0
while i<len(all_pixels):
    j=i+1
    while j<len(all_pixels) and all_pixels[j]==all_pixels[i] and j-i<255: j+=1
    rle.extend((j-i,all_pixels[i])); i=j
out=['/* Noto Sans CJK subset raster; SIL OFL-1.1, see assets/FONT-LICENSE. */',f'#define FONT_COUNT {len(chars)}','static const struct { unsigned cp; double advance; } font_glyphs[]={']
out.extend(f'{{{cp},{width}}},' for cp,width in glyphs)
out+=['};','static const unsigned char font_rle[]={']
out.extend(','.join(map(str,rle[i:i+32]))+',' for i in range(0,len(rle),32)); out+=['};']
(root/'assets/font_atlas.h').write_text('\n'.join(out)+'\n')
print(f'Baked {len(chars)} glyphs, {len(rle)} bytes')
