# IMAV 2022 arena asset provenance

The official simulator assets here were copied byte-for-byte from
[`tudelft/crazyflie-simulation`](https://github.com/tudelft/crazyflie-simulation),
branch `imav2022`, commit `da3636651e43ba7663eb3ed4f73c59f641058cef`.
They are the same pinned inputs used by the Isaac Lab IMAV environment.

`imav2022-gate.obj` is a deterministic, UV-preserving conversion of the pinned
official `imav2022-gate.dae`; its positions, normals, UVs, and 232 triangles are
retained. `feather_flag_proxy.obj` ports Isaac's nine-point silhouette and
`flag_white_blue_proxy_v1.png` is its locally derived, non-official texture.
The historical gate crop is retained for reproducibility,
but the scene renders official `GateTex.png` on the converted official mesh.

## SHA-256

```text
5643f7bea93f18bef334407efba8cc5cecdb0a26bd355e495cc062fdcefc4b64  GateTex.png
894db9b6b97aa4d7cce0c10138a0a1952a5c85653ae16e8dad539ae3eb378134  cabinets.png
8fdf17147fb5055e2514c906590a7d2a69be3f4408415e05c618fc177d88150e  flag_white_blue_proxy_v1.png
0f7a0c868b42b13f8fe28eabca7968c9bea15575b23345ca0a79cef7d934f0ad  feather_flag_proxy.obj
604f3fbf3316b2508b2ef66b04a65b898a78ccf11a2311de37c88d21900a3aa4  gate_orange_fabric_from_official_atlas.png
3c5059fea187f6b3ce05a53d9cbc67fe18f4247fb36560cd7719366370befc7d  grass_black.png
6782d555b7ff0333dd71173644f599bfd435643b95594a2ee723f27e0eefe52e  grass_blue.png
14afb8bc3961dcdee01a4f833fd68024177e6d30d4842cdc1c7fc4ff779e5a78  grass_green.png
afa65a0b5130a268a1a73f5a81cd039ddad78b05eb1afd236320cc7328f36be1  imav2022-gate.dae
881e0c58b918303e36db5cbeb09c8ef13c6f34541e8c84c0c43b8e51152f6806  imav2022-gate.obj
48a20e97f43d1049b7497b7b9ecd54b6b7ff04cc7b0241113a0c2c94e09d5ba2  mat_traffic_1.png
aba3c4fa41ed11e7ef7e3c75f53264b164e87865b3542905423ee259431358ab  metal_panel1.png
2801e1ff6dca49624bff5c7e6c55005dc2635e0b89a17d4c71576fb36de5a0e2  metal_panel3.png
c61f75bc61b38cf1bf3203c19961b1c1387c56965e389ee0bacf12377f0f469d  net.png
27574859a4969b59c24f856d19bda908771d3387f004ca1216ce4c3b3fc21eba  orange_pole.png
704a636049ffc4635349241c9b3747baaa056e4e96dacb77ab5d4351d64b55df  padded_pole.png
5e7795e46fd378f01689f151f9ae7e1f186e99c79da3a020607e7817ac79ddda  wall_curtains.png
```

## MIT license

Copyright (c) 2022 Bitcraze

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
