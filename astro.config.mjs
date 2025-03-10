import { defineConfig } from 'astro/config';
import tailwind from '@astrojs/tailwind';
import node from '@astrojs/node';

export default defineConfig({
  integrations: [tailwind()],
  site: 'https://winnież6.github.io',
  output: 'static',
  adapter: node({
    mode: 'standalone'
  })
}); 