/* =========================================================================
   NAVIGATION GUIDE — Dynamic Pathfinding & Terrain Trajectory System
   Calculates an obstacle-avoiding route from player/start to any target,
   projects it seamlessly onto Martian terrain relief, and exposes
   query functions to measure the player's distance to the path.
   ========================================================================= */

(function(){
  'use strict';

  // State
  let pathPoints = [];           // Array of THREE.Vector3 conforming to terrain
  let rawWaypoints = [];         // Key turn points from pathfinder
  let routeGroup = null;         // Three.js Group containing ribbon, line & markers
  let trailTexture = null;
  let activeTarget = null;       // { x, z }
  let activeStart = null;        // { x, z }
  let currentScene = null;
  let animationTime = 0;

  /* -------------------- Procedural Flow Texture -------------------- */
  function createTrailTexture(){
    if (trailTexture) return trailTexture;
    const canvas = document.createElement('canvas');
    canvas.width = 512;
    canvas.height = 64;
    const ctx = canvas.getContext('2d');

    // High-contrast holographic cyan glow with bright center beam
    const grad = ctx.createLinearGradient(0, 0, 0, 64);
    grad.addColorStop(0.0, 'rgba(0, 180, 255, 0)');
    grad.addColorStop(0.2, 'rgba(0, 210, 255, 0.45)');
    grad.addColorStop(0.5, 'rgba(255, 255, 255, 1.0)');
    grad.addColorStop(0.8, 'rgba(0, 210, 255, 0.45)');
    grad.addColorStop(1.0, 'rgba(0, 180, 255, 0)');
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, 512, 64);

    // Directional Chevrons (> > >) pointing toward objective
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 5;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    for (let x = 0; x < 512; x += 64){
      ctx.beginPath();
      ctx.moveTo(x + 10, 12);
      ctx.lineTo(x + 36, 32);
      ctx.lineTo(x + 10, 52);
      ctx.stroke();

      // Glowing core dot on apex
      ctx.fillStyle = '#4fd1ff';
      ctx.beginPath();
      ctx.arc(x + 36, 32, 6, 0, Math.PI * 2);
      ctx.fill();
    }

    trailTexture = new THREE.CanvasTexture(canvas);
    trailTexture.wrapS = THREE.RepeatWrapping;
    trailTexture.wrapT = THREE.ClampToEdgeWrapping;
    return trailTexture;
  }

  /* -------------------- Obstacle Collision Helpers -------------------- */
  function lineIntersectsObstacle(p1, p2, obstacle, margin){
    const r = obstacle.radius + (margin !== undefined ? margin : 0.85);
    const dx = p2.x - p1.x;
    const dz = p2.z - p1.z;
    const lenSq = dx * dx + dz * dz;
    if (lenSq === 0) return Math.hypot(p1.x - obstacle.x, p1.z - obstacle.z) < r;

    const t = Math.max(0, Math.min(1, ((obstacle.x - p1.x) * dx + (obstacle.z - p1.z) * dz) / lenSq));
    const projX = p1.x + t * dx;
    const projZ = p1.z + t * dz;
    const distSq = (obstacle.x - projX) * (obstacle.x - projX) + (obstacle.z - projZ) * (obstacle.z - projZ);
    return distSq < r * r;
  }

  function isSegmentClear(p1, p2, obstacles, margin){
    if (!obstacles || obstacles.length === 0) return true;
    for (let i = 0; i < obstacles.length; i++){
      if (lineIntersectsObstacle(p1, p2, obstacles[i], margin)) return false;
    }
    return true;
  }

  /* -------------------- 2D A* Pathfinding around rocks -------------------- */
  function findObstacleFreePath(start, target, obstacles){
    // 1. Direct path check (Fast path: if clear, straight line)
    if (isSegmentClear(start, target, obstacles, 0.9)){
      return [new THREE.Vector2(start.x, start.z), new THREE.Vector2(target.x, target.z)];
    }

    // 2. Discretized local grid A* around bounding box of start + target
    const minX = Math.min(start.x, target.x) - 16;
    const maxX = Math.max(start.x, target.x) + 16;
    const minZ = Math.min(start.z, target.z) - 16;
    const maxZ = Math.max(start.z, target.z) + 16;

    const cellSize = 1.2;
    const cols = Math.ceil((maxX - minX) / cellSize);
    const rows = Math.ceil((maxZ - minZ) / cellSize);

    const startCol = Math.max(0, Math.min(cols - 1, Math.round((start.x - minX) / cellSize)));
    const startRow = Math.max(0, Math.min(rows - 1, Math.round((start.z - minZ) / cellSize)));
    const targetCol = Math.max(0, Math.min(cols - 1, Math.round((target.x - minX) / cellSize)));
    const targetRow = Math.max(0, Math.min(rows - 1, Math.round((target.z - minZ) / cellSize)));

    const key = (c, r) => `${c},${r}`;
    const blocked = new Set();
    const closedSet = new Set();

    for (let i = 0; i < (obstacles || []).length; i++){
      const obs = obstacles[i];
      const radius = obs.radius + 0.85;
      const cMin = Math.max(0, Math.floor((obs.x - radius - minX) / cellSize));
      const cMax = Math.min(cols - 1, Math.ceil((obs.x + radius - minX) / cellSize));
      const rMin = Math.max(0, Math.floor((obs.z - radius - minZ) / cellSize));
      const rMax = Math.min(rows - 1, Math.ceil((obs.z + radius - minZ) / cellSize));

      for (let c = cMin; c <= cMax; c++){
        for (let r = rMin; r <= rMax; r++){
          const wx = minX + c * cellSize;
          const wz = minZ + r * cellSize;
          if (Math.hypot(wx - obs.x, wz - obs.z) < radius){
            blocked.add(key(c, r));
          }
        }
      }
    }

    blocked.delete(key(startCol, startRow));
    blocked.delete(key(targetCol, targetRow));

    const openSet = [{ col: startCol, row: startRow, g: 0, f: Math.hypot(startCol - targetCol, startRow - targetRow) }];
    const cameFrom = new Map();
    const gScore = new Map();
    gScore.set(key(startCol, startRow), 0);

    const neighbors = [
      { dc: 1, dr: 0, cost: 1.0 },
      { dc: -1, dr: 0, cost: 1.0 },
      { dc: 0, dr: 1, cost: 1.0 },
      { dc: 0, dr: -1, cost: 1.0 },
      { dc: 1, dr: 1, cost: 1.414 },
      { dc: -1, dr: 1, cost: 1.414 },
      { dc: 1, dr: -1, cost: 1.414 },
      { dc: -1, dr: -1, cost: 1.414 }
    ];

    let foundTarget = false;
    let iterations = 0;
    const maxIterations = 2000;

    while (openSet.length > 0 && iterations++ < maxIterations){
      openSet.sort((a, b) => a.f - b.f);
      const current = openSet.shift();
      const currentKey = key(current.col, current.row);

      if (current.col === targetCol && current.row === targetRow){
        foundTarget = true;
        break;
      }

      closedSet.add(currentKey);

      for (let i = 0; i < neighbors.length; i++){
        const n = neighbors[i];
        const nc = current.col + n.dc;
        const nr = current.row + n.dr;
        if (nc < 0 || nc >= cols || nr < 0 || nr >= rows) continue;
        const nKey = key(nc, nr);
        if (blocked.has(nKey) || closedSet.has(nKey)) continue;

        const tentG = (gScore.get(currentKey) || 0) + n.cost;
        if (tentG < (gScore.get(nKey) || Infinity)){
          cameFrom.set(nKey, { col: current.col, row: current.row });
          gScore.set(nKey, tentG);
          const h = Math.hypot(nc - targetCol, nr - targetRow);
          const f = tentG + h * 1.05;

          const existing = openSet.find(o => o.col === nc && o.row === nr);
          if (existing){
            existing.g = tentG;
            existing.f = f;
          } else {
            openSet.push({ col: nc, row: nr, g: tentG, f });
          }
        }
      }
    }

    const rawNodes = [];
    if (foundTarget){
      let curr = { col: targetCol, row: targetRow };
      const visitedBacktrack = new Set();
      while (curr){
        const cKey = key(curr.col, curr.row);
        if (visitedBacktrack.has(cKey)) break;
        visitedBacktrack.add(cKey);

        rawNodes.unshift(new THREE.Vector2(minX + curr.col * cellSize, minZ + curr.row * cellSize));
        if (curr.col === startCol && curr.row === startRow) break;
        curr = cameFrom.get(cKey);
      }
      rawNodes[0] = new THREE.Vector2(start.x, start.z);
      rawNodes[rawNodes.length - 1] = new THREE.Vector2(target.x, target.z);
    } else {
      return [new THREE.Vector2(start.x, start.z), new THREE.Vector2(target.x, target.z)];
    }

    // 3. String-pulling / Raycast Shortcut smoothing
    const smoothed = [rawNodes[0]];
    let anchorIdx = 0;
    let smoothSafety = 0;
    while (anchorIdx < rawNodes.length - 1 && smoothSafety++ < 500){
      let furthest = anchorIdx + 1;
      for (let j = rawNodes.length - 1; j > anchorIdx + 1; j--){
        if (isSegmentClear(rawNodes[anchorIdx], rawNodes[j], obstacles, 0.75)){
          furthest = j;
          break;
        }
      }
      smoothed.push(rawNodes[furthest]);
      anchorIdx = furthest;
    }

    return smoothed;
  }

  /* -------------------- Ribbon Geometry Generation -------------------- */
  function buildTerrainRibbonGeometry(points3D, heightFn, seed, width = 0.75){
    const numPoints = points3D.length;
    if (numPoints < 2) return new THREE.BufferGeometry();

    const positions = new Float32Array(numPoints * 2 * 3);
    const uvs = new Float32Array(numPoints * 2 * 2);
    const indices = [];

    let totalDist = 0;
    const dists = [0];
    for (let i = 1; i < numPoints; i++){
      totalDist += points3D[i].distanceTo(points3D[i - 1]);
      dists.push(totalDist);
    }

    for (let i = 0; i < numPoints; i++){
      const curr = points3D[i];
      let dir;
      if (i === 0){
        dir = points3D[1].clone().sub(curr).normalize();
      } else if (i === numPoints - 1){
        dir = curr.clone().sub(points3D[i - 1]).normalize();
      } else {
        dir = points3D[i + 1].clone().sub(points3D[i - 1]).normalize();
      }

      // Lateral vector perpendicular to route direction
      const side = new THREE.Vector3(-dir.z, 0, dir.x).normalize().multiplyScalar(width * 0.5);

      const pLeft = curr.clone().add(side);
      const pRight = curr.clone().sub(side);

      // Re-sample terrain height for each side vertex so slope never buries either side
      const yLeft = (typeof heightFn === 'function') ? heightFn(pLeft.x, pLeft.z, seed) + 0.22 : curr.y + 0.22;
      const yRight = (typeof heightFn === 'function') ? heightFn(pRight.x, pRight.z, seed) + 0.22 : curr.y + 0.22;

      const idx = i * 2;
      positions[idx * 3]     = pLeft.x;
      positions[idx * 3 + 1] = yLeft;
      positions[idx * 3 + 2] = pLeft.z;

      positions[(idx + 1) * 3]     = pRight.x;
      positions[(idx + 1) * 3 + 1] = yRight;
      positions[(idx + 1) * 3 + 2] = pRight.z;

      const u = dists[i] * 0.55;
      uvs[idx * 2]     = u;
      uvs[idx * 2 + 1] = 0.0;

      uvs[(idx + 1) * 2]     = u;
      uvs[(idx + 1) * 2 + 1] = 1.0;

      if (i < numPoints - 1){
        const a = idx;
        const b = idx + 1;
        const c = idx + 2;
        const d = idx + 3;
        indices.push(a, b, c);
        indices.push(b, d, c);
      }
    }

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geo.setAttribute('uv', new THREE.BufferAttribute(uvs, 2));
    geo.setIndex(indices);
    geo.computeVertexNormals();
    return geo;
  }

  /* -------------------- Main Navigation Module API -------------------- */
  const NavigationGuide = {
    enabled: true,

    /**
     * Compute and render a path from `start` to `target`.
     * Automatically projects onto terrain relief and avoids obstacles.
     */
    calculatePath: function(start, target, opts = {}){
      const scene = opts.scene || currentScene || window.surfaceScene;
      if (!scene) return [];
      currentScene = scene;

      const pPos = (typeof surfacePos !== 'undefined' ? surfacePos : (window.surfacePos || { x: 0, z: 0 }));
      const startPt = start || { x: pPos.x, z: pPos.z };

      const tMesh = opts.targetMesh || (typeof storyTargetMesh !== 'undefined' ? storyTargetMesh : window.storyTargetMesh);
      const targetPt = target || (tMesh ? { x: tMesh.position.x, z: tMesh.position.z } : null);

      if (!targetPt) return [];

      activeStart = { x: startPt.x, z: startPt.z };
      activeTarget = { x: targetPt.x, z: targetPt.z };

      const obstacles = opts.obstacles || (typeof surfaceRocksData !== 'undefined' ? surfaceRocksData : (window.surfaceRocksData || []));
      const seed = opts.seed || (typeof currentSurfaceSeed !== 'undefined' ? currentSurfaceSeed : (window.currentSurfaceSeed || 1));
      const heightFn = opts.terrainHeight || (typeof terrainHeight === 'function' ? terrainHeight : window.terrainHeight);

      // 1. Calculate optimal 2D node sequence avoiding obstacles
      rawWaypoints = findObstacleFreePath(activeStart, activeTarget, obstacles);

      // 2. Sample dense 3D points following terrain elevation
      pathPoints = [];
      if (rawWaypoints.length < 2){
        const yStart = (typeof heightFn === 'function') ? heightFn(activeStart.x, activeStart.z, seed) + 0.22 : 0.22;
        const yTarget = (typeof heightFn === 'function') ? heightFn(activeTarget.x, activeTarget.z, seed) + 0.22 : 0.22;
        pathPoints = [
          new THREE.Vector3(activeStart.x, yStart, activeStart.z),
          new THREE.Vector3(activeTarget.x, yTarget, activeTarget.z)
        ];
      } else {
        const waypoints3D = rawWaypoints.map(wp => {
          const y = (typeof heightFn === 'function') ? heightFn(wp.x, wp.y, seed) + 0.22 : 0.22;
          return new THREE.Vector3(wp.x, y, wp.y);
        });

        // Spline interpolation for smooth organic curve
        const curve = new THREE.CatmullRomCurve3(waypoints3D, false, 'centripetal', 0.4);
        const totalDist = waypoints3D.reduce((acc, pt, i) => i === 0 ? 0 : acc + pt.distanceTo(waypoints3D[i-1]), 0);
        const sampleCount = Math.max(20, Math.ceil(totalDist / 0.5)); // Sample every ~50cm
        const sampledPoints = curve.getPoints(sampleCount);

        pathPoints = sampledPoints.map(pt => {
          const ty = (typeof heightFn === 'function') ? heightFn(pt.x, pt.z, seed) + 0.22 : pt.y;
          return new THREE.Vector3(pt.x, ty, pt.z);
        });
      }

      this.renderRoute(scene, heightFn, seed);
      return pathPoints;
    },

    /**
     * Renders or updates the visual holographic ribbon + core line in Three.js scene.
     */
    renderRoute: function(targetScene, heightFn, seed){
      const scene = targetScene || currentScene || window.surfaceScene;
      if (!scene) return;
      currentScene = scene;

      if (routeGroup){
        scene.remove(routeGroup);
        routeGroup.traverse(child => {
          if (child.geometry) child.geometry.dispose();
          if (child.material) child.material.dispose();
        });
        routeGroup = null;
      }

      if (!this.enabled || pathPoints.length < 2) return;

      routeGroup = new THREE.Group();
      routeGroup.name = 'NavigationGuideGroup';

      // 1. Holographic glowing ribbon with moving chevrons
      const texture = createTrailTexture();
      const geo = buildTerrainRibbonGeometry(pathPoints, heightFn, seed, 0.75);

      const ribbonMat = new THREE.MeshBasicMaterial({
        map: texture,
        color: 0x4fd1ff,
        transparent: true,
        opacity: 0.92,
        side: THREE.DoubleSide,
        depthWrite: false,
        blending: THREE.AdditiveBlending,
        polygonOffset: true,
        polygonOffsetFactor: -4,
        polygonOffsetUnits: -4
      });

      const ribbonMesh = new THREE.Mesh(geo, ribbonMat);
      ribbonMesh.renderOrder = 999;
      routeGroup.add(ribbonMesh);

      // 2. High-intensity center laser spine line
      const lineGeo = new THREE.BufferGeometry().setFromPoints(
        pathPoints.map(p => new THREE.Vector3(p.x, p.y + 0.04, p.z))
      );
      const lineMat = new THREE.LineBasicMaterial({
        color: 0xc8f5ff,
        linewidth: 3,
        transparent: true,
        opacity: 0.85,
        blending: THREE.AdditiveBlending,
        depthWrite: false
      });
      const centerLine = new THREE.Line(lineGeo, lineMat);
      centerLine.renderOrder = 1000;
      routeGroup.add(centerLine);

      // 3. Periodic waypoint pulse nodes every ~3.5 meters
      const nodeGeo = new THREE.RingGeometry(0.12, 0.38, 16);
      nodeGeo.rotateX(-Math.PI / 2);
      const nodeMat = new THREE.MeshBasicMaterial({
        color: 0xffd15c,
        transparent: true,
        opacity: 0.75,
        side: THREE.DoubleSide,
        depthWrite: false,
        blending: THREE.AdditiveBlending
      });

      let distCounter = 0;
      for (let i = 1; i < pathPoints.length; i++){
        distCounter += pathPoints[i].distanceTo(pathPoints[i - 1]);
        if (distCounter >= 3.5){
          distCounter = 0;
          const nodeMesh = new THREE.Mesh(nodeGeo, nodeMat);
          nodeMesh.position.copy(pathPoints[i]);
          nodeMesh.position.y += 0.05;
          nodeMesh.renderOrder = 1001;
          routeGroup.add(nodeMesh);
        }
      }

      scene.add(routeGroup);
    },

    /**
     * Call every frame to animate the flowing chevrons toward the objective.
     */
    update: function(dt){
      const delta = dt || 0.016;
      animationTime += delta;

      if (trailTexture){
        trailTexture.offset.x -= delta * 1.85;
      }

      if (routeGroup){
        const pulse = 0.78 + Math.sin(animationTime * 3.5) * 0.18;
        routeGroup.children.forEach(child => {
          if (child.material && child.material.opacity !== undefined){
            child.material.opacity = pulse;
          }
        });
      }
    },

    /**
     * Force clear the route visual and geometry.
     */
    clear: function(){
      const scene = currentScene || window.surfaceScene;
      if (routeGroup && scene){
        scene.remove(routeGroup);
        routeGroup.traverse(child => {
          if (child.geometry) child.geometry.dispose();
          if (child.material) child.material.dispose();
        });
        routeGroup = null;
      }
      pathPoints = [];
      rawWaypoints = [];
      activeTarget = null;
    },

    /**
     * Returns an array of THREE.Vector3 points of the active path.
     */
    getPath: function(){
      return pathPoints;
    },

    /**
     * Returns the total route distance along the calculated path in meters.
     */
    getTotalDistance: function(){
      let dist = 0;
      for (let i = 1; i < pathPoints.length; i++){
        dist += pathPoints[i].distanceTo(pathPoints[i - 1]);
      }
      return dist;
    },

    /**
     * Computes the perpendicular/shortest distance from any position (default: player's surfacePos)
     * to the active navigation path ribbon.
     * Returns distance in meters.
     */
    getDistanceToPoint: function(pos){
      const p = pos || (typeof surfacePos !== 'undefined' ? surfacePos : (window.surfacePos || null));
      if (!p || pathPoints.length < 2) return Infinity;

      let minDistanceSq = Infinity;
      const px = p.x;
      const pz = p.z;

      for (let i = 0; i < pathPoints.length - 1; i++){
        const a = pathPoints[i];
        const b = pathPoints[i + 1];

        const dx = b.x - a.x;
        const dz = b.z - a.z;
        const lenSq = dx * dx + dz * dz;

        let t = 0;
        if (lenSq > 0.00001){
          t = Math.max(0, Math.min(1, ((px - a.x) * dx + (pz - a.z) * dz) / lenSq));
        }

        const projX = a.x + t * dx;
        const projZ = a.z + t * dz;
        const distSq = (px - projX) * (px - projX) + (pz - projZ) * (pz - projZ);

        if (distSq < minDistanceSq){
          minDistanceSq = distSq;
        }
      }

      return Math.sqrt(minDistanceSq);
    },

    /**
     * Shortcut for measuring current player distance from the route.
     */
    getDistanceToPlayer: function(){
      return this.getDistanceToPoint(typeof surfacePos !== 'undefined' ? surfacePos : (window.surfacePos || null));
    },

    /**
     * Returns detailed proximity info: closest point on path, distance, and route progress (0.0 to 1.0).
     */
    getClosestPointOnPath: function(pos){
      const p = pos || (typeof surfacePos !== 'undefined' ? surfacePos : (window.surfacePos || null));
      if (!p || pathPoints.length < 2){
        return { point: null, distance: Infinity, progress: 0 };
      }

      let minDistanceSq = Infinity;
      let closestPoint = new THREE.Vector3();
      let bestSegmentIdx = 0;
      let bestT = 0;

      const px = p.x;
      const pz = p.z;

      for (let i = 0; i < pathPoints.length - 1; i++){
        const a = pathPoints[i];
        const b = pathPoints[i + 1];

        const dx = b.x - a.x;
        const dz = b.z - a.z;
        const lenSq = dx * dx + dz * dz;

        let t = 0;
        if (lenSq > 0.00001){
          t = Math.max(0, Math.min(1, ((px - a.x) * dx + (pz - a.z) * dz) / lenSq));
        }

        const projX = a.x + t * dx;
        const projY = a.y + t * (b.y - a.y);
        const projZ = a.z + t * dz;
        const distSq = (px - projX) * (px - projX) + (pz - projZ) * (pz - projZ);

        if (distSq < minDistanceSq){
          minDistanceSq = distSq;
          closestPoint.set(projX, projY, projZ);
          bestSegmentIdx = i;
          bestT = t;
        }
      }

      let distSoFar = 0;
      let totalDist = 0;
      for (let i = 0; i < pathPoints.length - 1; i++){
        const segLen = pathPoints[i].distanceTo(pathPoints[i + 1]);
        if (i < bestSegmentIdx){
          distSoFar += segLen;
        } else if (i === bestSegmentIdx){
          distSoFar += segLen * bestT;
        }
        totalDist += segLen;
      }

      const progress = totalDist > 0 ? Math.min(1, Math.max(0, distSoFar / totalDist)) : 0;
      return {
        point: closestPoint,
        distance: Math.sqrt(minDistanceSq),
        progress: progress
      };
    },

    /**
     * Checks if player is within a specific deviation tolerance of the path.
     */
    isPlayerOnTrack: function(tolerance = 3.5){
      return this.getDistanceToPlayer() <= tolerance;
    }
  };

  // Expose global accessors
  window.NavigationGuide = NavigationGuide;
  window.getDistanceToNavigationPath = function(pos){
    return NavigationGuide.getDistanceToPoint(pos);
  };
})();
