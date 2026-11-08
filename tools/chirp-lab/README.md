# Chirp Lab

An interactive view of one received frame going through the firmware's chirp
decoder. Open `index.html` in a browser (double-click is fine; no server or
build step needed).

## What it shows

One section per stage of `chirp_decoder.c`:

1. **Received signal**: a repeated symbol *k* (a cyclically shifted real
   up-chirp), with a window offset, a time scale (clock mismatch or Doppler)
   and white noise.
2. **Band-pass**: FFT, keep only the chirp band's positive-frequency bins,
   inverse FFT. Gives a complex band-limited signal.
3. **De-chirp**: multiply by the conjugate base chirp and FFT. The shifted
   chirp wraps part way through the frame, so it lands in two bins, *kL* and
   *kL − B*.
4. **Fold**: add the two pieces as complex values after a fixed rotation *R*
   (the firmware), compared with the old power sum.
5. **Decide**: the strongest bin gives the symbol; mean shift refines its
   position (the timing error); the circular centroid gives magnitude and
   strength.

The decoder switches (band-pass on/off, coherent or power-sum fold) let you
compare the current decoder with the previous one.

Settings can be put in the URL to save or share a case, using the control
names from `CONTROLS` in `app.js`, e.g.
`index.html?k=20&offset=3&snr=-10&ppm=1000&bandPass=false`.

## Files

| File | Contents |
|---|---|
| `index.html` | Layout: the control rack placeholder, readouts and one `<figure>` per plot. |
| `style.css` | Look and feel; colours are CSS custom properties, with a dark theme. |
| `dsp.js` | The signal processing, as pure functions with no DOM. Each function names the firmware function it mirrors. Also loadable from Node. |
| `charts.js` | Generic canvas charts (`xy`, `polar`) with hover readouts. Knows nothing about chirps. |
| `app.js` | The controls table (`CONTROLS`), state, and one function per stage that turns the decoder's output into chart descriptions. |
| `check.html` | Checks `dsp.js` against numbers from the C host tests. Open it after changing `dsp.js`. |

## Keeping it in step with the firmware

When the decoder changes:

1. Change the matching function in `dsp.js` (the header lists which is which).
2. Run the C host tests (`ctest --test-dir tests/build -V`) and update the
   expected values in `check.html` from their output.
3. Open `check.html`; every check should pass.

## Common changes

- **New parameter**: add an entry to `CONTROLS` in `app.js` (label, type,
  default, optional slider range). It appears in the rack and as `state.<key>`.
  Then use it in `render()` or a stage function.
- **New plot**: add a `<figure id="...">` with a `<canvas>` to `index.html`,
  then call `ChirpCharts.xy(figure, cfg)` or `ChirpCharts.polar(figure, data)`
  from a stage function in `app.js`. The `cfg` format is described at the top
  of `charts.js`.
- **New readout**: add a `<div class="ro">` to `index.html` and set it in
  `readouts()` in `app.js`.

Checks can also be run headless, e.g.:

```
google-chrome --headless=new --dump-dom "file://$PWD/tools/chirp-lab/check.html" | grep summary
```
