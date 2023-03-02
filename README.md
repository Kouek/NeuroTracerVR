# Neuron Tracer in VR

## Architeture

## Space

- Word Space --- ( * preScale ) -- ( + preTranslate ) -->
- Volume Render Space --- ( / sapces ) -->
- Volume Space

## FAVR (Focus Adaptive Volume Rendering)

- Rendering size $w \times h$ on the full screen should meet $w=h$.
- Denote $hw$ as half of $w$.
- Then, using Subsample Level $L$, we have
  - In Reconstruction Lookup Texture, radiuses separating each $stage$ are
    - $0, hw/L, 2*hw/L, ..., (L-1)*hw/L, +\infty$
  - In Subsample Lookup Texture, radius range of each $stage$ is
    - $[0,\ hw/L]$, when $stage=0$
    - $[(1-scale)*hw/L,\ hw/L]$, $scale=1-stage/L$, when $stage \in [1,L-2]$
    - $[(1-scale)*hw/L, \ +\infty)$, $scale=1/L$, when $stage=L-1$
