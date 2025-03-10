import { defineConfig } from 'astro/config';
import tailwind from '@astrojs/tailwind';

export default defineConfig({
  site: 'https://winnież6.github.io',
  integrations: [tailwind()],
  output: 'static'
}); 