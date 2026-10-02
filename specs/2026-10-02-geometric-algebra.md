# Geometric algebra (3D PGA)

`2026-10-02` · Steven Otto · PR: feat/geometric-algebra

## Waarom

Rotaties en translaties zitten nu verspreid over `quat`, `quat2` en `mat4`, en punten, lijnen en vlakken hebben elk hun eigen ad-hoc representatie (`Vec3`, `Plane3`, geen lijn). Projectieve geometrische algebra (PGA, Cl(3,0,1)) geeft één algebra waarin punten, lijnen, vlakken en bewegingen (motors) dezelfde datastructuur delen en met een handvol producten (geometrisch, wedge, vee, sandwich) te combineren zijn: snijden, verbinden, projecteren en transformeren zonder speciale gevallen. Bibliotheken als ganja.js bewijzen de aanpak in JS, maar zijn klasse- en allocatie-gebaseerd en passen niet in de data-georiënteerde, allocatievrije stijl van `math`. Zonder deze module blijven gebruikers die GA willen inzetten aangewezen op een tweede library met een andere datastijl.

## Scope

- **Wel:** een nieuw subpad `math/ga` met de namespace `pga3d`: een 16-componenten multivector als platte tuple, de standaardproducten en -involuties, exp/log voor motors, constructors voor punt/vlak/lijn, conversie van en naar `quat2`, `mat4` en `Vec3`, unit tests, een micro-bench en registratie in de docs-pijplijn.
- **Niet:** 2D PGA, conforme GA (CGA), gespecialiseerde kleinere types (losse Motor/Point/Plane-arrays), code-generatie voor willekeurige signaturen, een voorbeeld in de examples-gallery.

## Acceptatiecriteria

1. `import { pga3d, type PGA3D } from 'math/ga'` werkt via package exports, rollup, typedoc en de docs-generator (README/API.md).
2. Een `PGA3D` is een platte tuple van 16 getallen in de bivector.net-volgorde `[1, e0, e1, e2, e3, e01, e02, e03, e12, e31, e23, e021, e013, e032, e123, e0123]`.
3. Alle functies volgen de `out`-eerst-conventie, zijn allocatievrij en veilig bij aliasing (`pga3d.mul(a, a, b)`).
4. Het geometrisch product, de wedge en de vee reproduceren de Cayley-tabel van ganja.js voor `Algebra(3,0,1)` op willekeurige multivectors (getest tegen handmatig geverifieerde referentiewaarden).
5. `exp` van een bivector levert een genormaliseerde motor en `log(exp(B)) ≈ B` voor eenvoudige en gemengde (schroef-)bivectors.
6. `fromQuat2`/`toQuat2` en `fromMat4` zijn consistent met `quat2` en `mat4`: hetzelfde punt transformeren via motor-sandwich of via `vec3.transformMat4` geeft hetzelfde resultaat.
7. `wedge(plane, plane)` geeft de snijlijn, `vee(point, point)` de verbindingslijn en `wedge(line, plane)` het snijpunt, geverifieerd op bekende configuraties.
8. `pnpm test`, `pnpm lint`, `pnpm format`, `pnpm build` en `pnpm docs` slagen zonder nieuwe waarschuwingen.

## Domeinmodel

- **Multivector**: som van geschaalde basisblades van graad 0 t/m 4. Eén type, `PGA3D`.
- **Vlak** (graad 1): `a e1 + b e2 + c e3 + d e0`, het vlak `ax + by + cz + d = 0`.
- **Lijn** (graad 2): Plücker-coördinaten, richting in `e23, e31, e12`, moment in `e01, e02, e03`.
- **Punt** (graad 3): `x e032 + y e013 + z e021 + e123`, homogene coördinaat op `e123`.
- **Motor**: even multivector (graad 0, 2, 4) met norm 1, de PGA-tegenhanger van `Quat2`. Transformeert elk element via het sandwich-product `M X ~M`.

## Open vragen

- Is één volledig 16-componenten type voldoende voor de eerste versie, of zijn gespecialiseerde kleinere types (zoals Klein doet) nodig voor performance? Aanname: één type eerst, meten voordat we specialiseren.
- Hoort 2D PGA (`pga2d`) in een vervolg-PR? Aanname: ja.
