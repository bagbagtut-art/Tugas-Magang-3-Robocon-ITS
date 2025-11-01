#  Turtle Tracer ROS2 — Logo Bocil Crypto

<img width="212" height="238" alt="image" src="https://github.com/user-attachments/assets/572c1709-81b6-4e65-ac9c-080a2a1ba5e2" />
<img width="219" height="231" alt="image" src="https://github.com/user-attachments/assets/0957e883-a652-4d5a-ab74-5e696102418a" />

Proyek ini menggunakan **ROS2 (Humble)** dan **Turtlesim** untuk menggambar pola spiral yang mengikuti bentuk **logo Akademi Crypto**.  
Program membaca gambar logo, lalu menggunakan nilai intensitas piksel sebagai masker.  
Hasilnya: turtle hanya menggambar di area terang atau gelap sesuai pengaturan parameter `--logo-is-dark`.

---

##  Fitur Utama
- Menggambar **spiral otomatis** di simulasi Turtlesim.
- Menggunakan **gambar PNG** sebagai **mask** untuk menentukan area gambar.
- Dapat mengatur:
  - Warna garis spiral (`--pen-spiral`)
  - Warna garis awal (`--pen-y`)
  - Kecepatan rotasi dan jarak spiral (`--spiral-k`, `--spiral-step`)
- Posisi turtle otomatis dimulai dari tengah logo.

---

##  Struktur Proyek


```text
ros2_ws/
├── src/
│   └── turtle_tracer_ros2/
│       ├── package.xml
│       ├── setup.cfg
│       ├── setup.py
│       └── turtle_tracer_ros2/
│           ├── __init__.py
│           └── draw_logo_masked_spiral.py
```


---

##  Instalasi

1. **Masuk ke workspace**
   ```bash
   cd ~/ros2_ws
2. **Build paket**
    ```bash
   colcon build --symlink-install
3. **Source environment**
    ```bash
   source /opt/ros/humble/setup.bash
    source install/setup.bash
4. **Jalankan Turtlesim**
    ```bash
   ros2 run turtlesim turtlesim_node
5. **Menjalankan Turtle Tracer**
   ```bash
    ros2 run turtle_tracer_ros2 draw_logo_masked_spiral --clear \
    --logo /home/user/Gemini_Generated_Image_h8mkz1h8mkz1h8mk.png \
    --px-per-unit 50 --logo-rot 15 --logo-cx 5.5 --logo-cy 5.5 \
    --threshold 205 --logo-is-dark \
   --spiral-k 0.0475 --spiral-step 0.010 --sleep 0.0015 \
    --pen-y 170,100,255,6 --pen-spiral 255,255,255,3

---

## Penjelasan Singkat Logika kode
1. draw_logo_masked_spiral.py
  Node utama yang:
  -Membaca gambar logo (pakai Pillow)
  -Mengonversi ke grayscale
  -Mengontrol turtle lewat service /clear, /set_pen, dan /teleport_absolute
  -Menggambar spiral dengan persamaan polar (r, θ) berdasarkan warna piksel
   
2. setup.py
  Mendaftarkan entry point draw_logo_masked_spiral sebagai executable node ROS2.

3. package.xml
  Mendefinisikan dependensi seperti rclpy, turtlesim, dan python3-pil.

---

## Lisensi
Proyek ini dibuat untuk keperluan pembelajaran ROS2 dan simulasi robotik.
Bebas digunakan untuk riset, tugas akhir, atau eksperimen pribadi.

---

## Hasil Simulasi
<img width="1303" height="877" alt="image" src="https://github.com/user-attachments/assets/13a48dbc-cf10-4a5e-bfd4-090190b9bd7c" />

---

```text
Burung pipit ada di sawah 
Burung enggang di Kalimantan Tengah 
Mohon maaf kalau ada salah 
Namanya juga manusia
```












