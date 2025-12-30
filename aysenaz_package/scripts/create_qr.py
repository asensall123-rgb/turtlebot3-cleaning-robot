#!/usr/bin/env python3
import qrcode
import os

# QR kodu verileri (odalar için)
rooms = {
    "LIVINGROOM": "ROOM=LIVINGROOM",
    "KITCHEN": "ROOM=KITCHEN",
    "BEDROOM": "ROOM=BEDROOM",
    "BATHROOM": "ROOM=BATHROOM"
}


texture_path = "/home/ayse/aysen_ws/src/aysenaz_package/media/materials/textures"
os.makedirs(texture_path, exist_ok=True)

# QR kodlarını oluştur ve kaydet
for room, data in rooms.items():
    qr = qrcode.QRCode(
        version=1,                  # QR boyutu otomatik
        error_correction=qrcode.constants.ERROR_CORRECT_L,
        box_size=10,                 # Her kare boyutu
        border=4                     # QR kenar boşluğu
    )
    qr.add_data(data)
    qr.make(fit=True)

    img = qr.make_image(fill='black', back_color='white')
    img_file = os.path.join(texture_path, f"{room.lower()}.png")
    img.save(img_file)
    print(f"{room} QR kodu oluşturuldu ve kaydedildi: {img_file}")

