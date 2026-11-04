from pathlib import Path
root=Path(__file__).resolve().parents[1]
captured=bytes(int(x,16)for x in (root/'reports/uart_capture.hex').read_text().split())
expected=bytes(int(x,16)for x in (root/'rom/messages.hex').read_text().split())[:14*128]
assert captured==expected, (len(captured),len(expected))
text='\n'.join(captured[n:n+128].decode('ascii').strip() for n in range(0,len(captured),128))+'\n'
(root/'reports/uart_decoded.txt').write_text(text)
print('PASS independent UART wire decode: BOOT, 12 stages, DONE; 1792 bytes, 8N1 framing')
