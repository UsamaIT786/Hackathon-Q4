---
title: Professional UI Components Showcase
description: Examples of all professional UI components and hover effects
---

# Professional UI Components Showcase

## Button Components

### Gradient Button with Ripple Effect
Click the button below to see the ripple and glow effect:

```html
<button class="button-gradient">
  Get Started with AI Robotics
</button>
```

**Effects on Hover:**
- Lifts 3px higher
- White ripple expands from center
- Glow shadow appears
- Smooth 300ms transition

---

## Image Components

### Image Card with Overlay

```html
<div class="image-card">
  <img src="ai-generated-robot.jpg" alt="Humanoid Robot">
  <div class="image-card-overlay">
    <p>Advanced Robotics Technology</p>
  </div>
</div>
```

**Effects on Hover:**
- Image scales 108% with slight rotation
- Overlay fades in (0% → 100% opacity)
- Brightness increases by 10%
- Saturation increases by 10%

### Featured Image

```html
<div class="featured-image">
  <img src="ai-robot-hero.jpg" alt="Featured Robot">
</div>
```

**Effects on Hover:**
- Lifts 4px up
- Rotates -1 degree
- Scales 105%
- Brightness and contrast increase

### Floating Image (Continuous Animation)

```html
<img src="robot-illustration.jpg" class="image-float" alt="Floating Robot">
```

**Effect:** Continuously floats up and down (15px range) over 4 seconds

---

## Card Components

### Feature Card with Shimmer Effect

```html
<div class="feature-card">
  <h3>Sim-to-Real Transfer</h3>
  <p>Train your models in simulation and deploy on real hardware with confidence using domain randomization techniques.</p>
</div>
```

**Effects on Hover:**
- Lifts 6px
- Border changes to primary color
- Shimmer effect sweeps left to right
- Shadow elevation increases

---

## Text Effects

### Gradient Text

```html
<span class="gradient-text">Physical AI and Humanoid Robotics</span>
```

**Effect:** Blue to Green gradient with bold weight (700)

### Highlighted Text with Animation

```html
<span class="highlight-animated">cutting-edge technology</span>
```

**Effect on Hover:**
- Background highlight expands
- Opacity increases
- Smooth 300ms transition

### Link with Underline Animation

```html
<a href="/docs" class="underline-hover">Learn more about ROS 2</a>
```

**Effect on Hover:**
- Gradient underline expands from left to right
- Smooth 150ms animation

---

## Hero Section with Parallax

```html
<div class="hero-section" style="background-image: url('ai-robot-landscape.jpg');">
  <div class="hero-section-content">
    <h1 class="gradient-text">Welcome to Physical AI</h1>
    <p>From Digital Intelligence to Embodied Autonomous Systems</p>
    <button class="button-gradient">Explore Chapters</button>
  </div>
</div>
```

**Features:**
- Parallax background (fixed attachment)
- 85% opacity gradient overlay (Blue → Green)
- Radial light gradient for depth
- Centered content with padding
- 500px minimum height
- Premium shadow (2xl)

---

## Icon Button with Rotation

```html
<button class="icon-button">🚀</button>
```

**Effects on Hover:**
- Scales 115%
- Rotates 15 degrees
- Color changes to darker shade
- Glow shadow appears

---

## Image Utilities

### Responsive Image with Shadow

```html
<img src="image.jpg" class="img-responsive img-shadow img-rounded" alt="Description">
```

Classes:
- `.img-responsive` - Max-width 100%, responsive
- `.img-shadow` - Box shadow with border radius
- `.img-rounded` - 12px border radius
- `.img-bordered` - Primary color border

---

## Complete Example: Feature Section

```html
<div style="margin: 3rem 0;">
  <h2>Why Choose Physical AI Learning</h2>
  
  <div class="feature-card">
    <h3>🤖 Comprehensive Curriculum</h3>
    <p>Learn everything from sensors and actuators to advanced control algorithms</p>
  </div>

  <div class="feature-card">
    <h3>🎯 Hands-On Projects</h3>
    <p>Build real robots and deploy AI models with step-by-step guidance</p>
  </div>

  <div class="feature-card">
    <h3>📚 Industry-Ready Content</h3>
    <p>Access the same techniques used by leading robotics companies</p>
  </div>
</div>
```

---

## Color Palette

The professional color scheme uses:

```
Primary Blue:    #0052CC (main actions and focus)
Secondary Green: #10B981 (success and accents)
Info Cyan:       #06B6D4 (information)
Warning Amber:   #F59E0B (warnings)
Danger Red:      #EF4444 (errors)
```

All colors automatically adapt to dark mode with optimized variants.

---

## Animation Timing Functions

- **Fast (150ms)**: Micro-interactions and quick hovers
- **Smooth (300ms)**: Page transitions and card animations
- **Bounce (400ms)**: Celebratory or attention-grabbing effects

All use smooth cubic-bezier easing: `cubic-bezier(0.4, 0, 0.2, 1)`

---

## Dark Mode Support

All components automatically support dark mode:

```
Light Mode Background: Linear gradient (#f5f7fa → #c3cfe2)
Dark Mode Background:  Linear gradient (#1a202c → #2d3748)
```

Colors automatically switch for optimal contrast and readability.

---

## Accessibility Features

- ✅ WCAG AA contrast ratios maintained
- ✅ Focus states clearly visible
- ✅ Touch targets ≥ 48px on mobile
- ✅ Semantic HTML structure
- ✅ Descriptive alt text on images
- ✅ Respects `prefers-reduced-motion`

---

## Mobile Optimization

On devices ≤768px:
- Reduced hover animations (4% scale instead of 8%)
- Touch-friendly button sizes (44-48px)
- Simplified effects for better performance
- Adjusted spacing and padding

---

## Performance Characteristics

- **GPU Accelerated**: Uses CSS transforms
- **Smooth 60fps**: Optimized transitions
- **Minimal Repaints**: Grouped hover states
- **Efficient Shadows**: Pre-calculated values
- **Lazy Loading Ready**: Works with image lazy loading

---

## Integration with Docusaurus

These components work seamlessly with Docusaurus markdown:

```markdown
### Using Feature Cards in Markdown

<div class="feature-card">

#### Advanced Robotics

Learn state-of-the-art techniques in humanoid robot development.

</div>

### Using Image Components

<div class="image-card">
  <img src="path/to/ai-image.jpg" alt="AI Generated Image">
  <div class="image-card-overlay">
    Click to learn more
  </div>
</div>

### Using Gradient Text

<span class="gradient-text">Powerful Learning Experience</span>
```

---

## Best Practices

1. **Use semantic button elements** for accessibility
2. **Always provide alt text** on images
3. **Test on mobile** - animations should feel responsive
4. **Respect user preferences** for motion
5. **Use contrast checkers** to ensure readability
6. **Load AI images responsively** - use srcset for multiple sizes
7. **Lazy load images** - add loading="lazy"

---

**Status**: ✅ Production Ready
**Last Updated**: 2025-12-21
**Browser Support**: All modern browsers

