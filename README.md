# Tucil 2 - Voxelisasi 3D Mesh dengan Octree

## Tentang Program

Program ini merupakan tugas untuk meengubah file 3D mesh (.obj) menjadi bentuk voxel (seperti dalam game minecraft) menggunakan octree. Cara kerjanya, ruang 3D dibagi-bagi jadi 8 bagian secara rekursif (divide and conquer) sampai kedalaman yang ditentukan. Bagian yang terdapat segitiga mesh bakal jadi voxel, terus disimpan lagi sebagai file .obj.

untuk memaksimalkan program agar berjalan lebih cepat, kami mengimplementasikan concurrency dengan OpenMP untuk paralelisasi proses build octree.

## Requirement

- g++ (support C++11 ke atas)
- OpenMP (biasanya udah include di g++)

Kalau belum ada g++:

**Windows:**
```
winget install MinGW.MinGW
```

**Linux:**
```
sudo apt install g++
```

## Kompilasi

Dari folder `Tucil2_13524102_13524085/`:

```bash
g++ src/main.cpp -o bin/main.exe -fopenmp
```

Jangan lupa `-fopenmp`-nya, kalau ga ntar OpenMP-nya ga jalan.

## Cara Pakai

Jalankan dari folder `Tucil2_13524102_13524085/`:

```bash
./bin/main.exe
```

Selama program berjalan akan meminta:

1. **Nama file input** — file .obj yang ada di `test/input/`. Boleh nulis `cow` atau `cow.obj`, dua-duanya bisa.

2. **Kedalaman octree** — semakin dalam semakin detail

3. **Nama file output** — nama file hasil. Boleh nulis `hasil` atau `hasil.obj`. Otomatis bakal jadi `hasil_voxel.obj` dan disimpan di `test/output/`.

4. Setelah selesai mengkonfirmasi.

### File input yang tersedia

Di `test/input/` udah ada beberapa file buat dicoba:
- cow.obj
- pumpkin.obj
- teapot.obj
- tetrahedron.obj
- torus.obj
- cube_clean.obj

## Struktur Folder

```
Tucil2_13524102_13524085/
├── bin/
│   └── main.exe
├── src/
│   └── main.cpp
└── test/
    ├── input/
    └── output/
```

## Author

| Nama | NIM |
|------|-----|
| Manuel Thimoty Silalahi | 13524102 |
| Ariel Cornelius Sitorus | 13524085 |
