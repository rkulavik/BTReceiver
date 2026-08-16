const fs = require('fs');
const path = require('path');

const rootDir = __dirname;
const distDir = path.join(rootDir, 'dist');

console.log('Building BTReceiver static distribution...');

// Ensure dist directory exists and is clean
if (fs.existsSync(distDir)) {
  fs.rmSync(distDir, { recursive: true, force: true });
}
fs.mkdirSync(distDir, { recursive: true });

// List of files and folders to include in distribution
const entriesToCopy = [
  'index.html',
  'app.js',
  'dsp-worker.js',
  'styles.css',
  'README.md'
];

entriesToCopy.forEach(entry => {
  const src = path.join(rootDir, entry);
  const dest = path.join(distDir, entry);

  if (fs.existsSync(src)) {
    const stat = fs.statSync(src);
    if (stat.isDirectory()) {
      fs.cpSync(src, dest, { recursive: true });
      console.log(`✓ Copied directory: ${entry} -> dist/${entry}`);
    } else {
      fs.copyFileSync(src, dest);
      console.log(`✓ Copied file: ${entry} -> dist/${entry}`);
    }
  } else {
    console.warn(`! Warning: Entry not found: ${entry}`);
  }
});

console.log('Build completed successfully! Artifacts placed in /dist directory.');
