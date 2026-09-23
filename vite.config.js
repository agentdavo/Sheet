import { defineConfig } from 'vite';

// Addons import 'three'; alias it to the WebGPU build so a single copy of the
// core classes is shared between the renderer and the addons.
export default defineConfig({
  base: './',
  resolve: {
    alias: [{ find: /^three$/, replacement: 'three/webgpu' }],
  },
  worker: { format: 'es' },
  build: { target: 'es2022', chunkSizeWarningLimit: 2000 },
});
